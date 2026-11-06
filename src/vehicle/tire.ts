/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import { clamp } from '../utils/math';

export type SurfaceType = 'asphalt' | 'dirt' | 'mud' | 'gravel';

export interface PacejkaCoefficients {
  muX: number;       // Peak longitudinal friction coefficient
  muY: number;       // Peak lateral friction coefficient
  Bx: number;        // Stiffness factor (longitudinal)
  Cx: number;        // Shape factor (longitudinal)
  Ex: number;        // Curvature factor (longitudinal)
  By: number;        // Stiffness factor (lateral)
  Cy: number;        // Shape factor (lateral)
  Ey: number;        // Curvature factor (lateral)
  rollingResCoeff: number; // Rolling resistance coefficient Crr
}

// Empirical Pacejka coefficient sets tuned for off-road vintage 4x4 tires
export const SURFACE_PACEJKA: Record<SurfaceType, PacejkaCoefficients> = {
  asphalt: {
    muX: 1.05,
    muY: 1.00,
    Bx: 10.5,
    Cx: 1.65,
    Ex: 0.95,
    By: 9.5,
    Cy: 1.35,
    Ey: -1.0,
    rollingResCoeff: 0.015,
  },
  dirt: {
    muX: 0.82,
    muY: 0.78,
    Bx: 8.5,
    Cx: 1.45,
    Ex: 0.90,
    By: 7.8,
    Cy: 1.25,
    Ey: -1.1,
    rollingResCoeff: 0.035,
  },
  mud: {
    muX: 0.38,
    muY: 0.34,
    Bx: 5.2,
    Cx: 1.20,
    Ex: 0.75,
    By: 4.8,
    Cy: 1.10,
    Ey: -1.4,
    rollingResCoeff: 0.090,
  },
  gravel: {
    muX: 0.58,
    muY: 0.52,
    Bx: 6.8,
    Cx: 1.30,
    Ex: 0.85,
    By: 6.0,
    Cy: 1.15,
    Ey: -1.2,
    rollingResCoeff: 0.060,
  },
};

export interface TireEvaluationResult {
  Fx: number;               // Longitudinal force along wheel heading (N)
  Fy: number;               // Lateral force along axle (N)
  slipRatio: number;        // Longitudinal slip kappa
  slipAngleRad: number;     // Lateral slip angle alpha (radians)
  slipAngleDeg: number;     // Lateral slip angle alpha (degrees)
  gripUtilization: number;  // Combined friction ellipse usage [0 .. 1.0]
  rollingResForce: number;  // Rolling resistance opposing wheel (N)
  aligningTorque: number;   // Self-aligning torque Mz (N*m)
}

export class TireModel {
  public static readonly NOMINAL_FZ = 4415.0;
  public static readonly LOAD_SENSITIVITY = 0.12;

  /**
   * Evaluates tire contact forces given kinematic slip and vertical load.
   */
  public static evaluate(
    slipRatio: number,
    slipAngleRad: number,
    normalLoadFz: number,
    surface: SurfaceType
  ): TireEvaluationResult {
    if (normalLoadFz <= 1.0) {
      return {
        Fx: 0,
        Fy: 0,
        slipRatio,
        slipAngleRad,
        slipAngleDeg: (slipAngleRad * 180) / Math.PI,
        gripUtilization: 0,
        rollingResForce: 0,
        aligningTorque: 0,
      };
    }

    const coeffs = SURFACE_PACEJKA[surface] ?? SURFACE_PACEJKA.dirt;

    // Load Sensitivity
    const loadRatio = (normalLoadFz - this.NOMINAL_FZ) / this.NOMINAL_FZ;
    const loadFactor = clamp(1.0 - this.LOAD_SENSITIVITY * loadRatio, 0.70, 1.25);

    const peakMuX = coeffs.muX * loadFactor;
    const peakMuY = coeffs.muY * loadFactor;

    const Dx = peakMuX * normalLoadFz;
    const Dy = peakMuY * normalLoadFz;

    // Pure Longitudinal Force Fx0(kappa)
    const BxK = coeffs.Bx * slipRatio;
    const atanBxK = Math.atan(BxK);
    const argX = coeffs.Cx * Math.atan(BxK - coeffs.Ex * (BxK - atanBxK));
    const Fx0 = Dx * Math.sin(argX);

    // Pure Lateral Force Fy0(alpha) - Note negative sign: opposes positive alpha
    const ByA = coeffs.By * slipAngleRad;
    const atanByA = Math.atan(ByA);
    const argY = coeffs.Cy * Math.atan(ByA - coeffs.Ey * (ByA - atanByA));
    const Fy0 = -Dy * Math.sin(argY);

    // Friction Ellipse Combined Slip Coupling
    const normFx = Fx0 / Math.max(1.0, Dx);
    const normFy = Fy0 / Math.max(1.0, Dy);
    const ellipseRadius = Math.sqrt(normFx * normFx + normFy * normFy);

    let finalFx = Fx0;
    let finalFy = Fy0;
    let gripUtilization = ellipseRadius;

    if (ellipseRadius > 1.0) {
      finalFx = Fx0 / ellipseRadius;
      finalFy = Fy0 / ellipseRadius;
      gripUtilization = 1.0;
    }

    // Rolling Resistance Force
    const rollingResForce = coeffs.rollingResCoeff * normalLoadFz;

    // Pneumatic trail aligning torque Mz
    const pneumaticTrail = 0.04 * (1.0 - clamp(Math.abs(slipAngleRad) / 0.35, 0, 0.9));
    const aligningTorque = -finalFy * pneumaticTrail;

    return {
      Fx: finalFx,
      Fy: finalFy,
      slipRatio,
      slipAngleRad,
      slipAngleDeg: (slipAngleRad * 180) / Math.PI,
      gripUtilization,
      rollingResForce,
      aligningTorque,
    };
  }

  /**
   * Slip calculation:
   * Longitudinal slip kappa = (omega * r - vx) / max(vMin, |vx|).
   * Notice: kappa depends directly on the relative slip speed (omega * r - vx).
   * When stationary and wheel is stationary (omega = 0, vx = 0), kappa is exactly 0.
   * When wheel spins during a burnout (omega > 0, vx = 0), kappa is positive and generates tractive force.
   */
  public static computeSlip(
    vx: number,         // Longitudinal contact speed (m/s)
    vy: number,         // Lateral contact speed (m/s)
    wheelOmega: number, // Wheel angular velocity (rad/s)
    wheelRadius: number // Wheel rolling radius (m)
  ): { kappa: number; alpha: number } {
    const wheelTangentialSpeed = wheelOmega * wheelRadius;
    const slipSpeedLong = wheelTangentialSpeed - vx;

    const vMin = 0.50;
    const vLongRef = Math.max(vMin, Math.abs(vx));

    let rawKappa = 0;
    if (Math.abs(slipSpeedLong) > 0.001) {
      rawKappa = slipSpeedLong / vLongRef;
    }

    // Blend slip ratio to zero at true standstill (both road speed and wheel speed near zero).
    // When wheel is spinning during launch or burnout, effectiveSpeed is high and longBlend = 1.0.
    const totalGroundSpeed = Math.sqrt(vx * vx + vy * vy);
    const effectiveSpeed = Math.max(totalGroundSpeed, Math.abs(wheelTangentialSpeed));
    const longBlend = clamp(effectiveSpeed / 0.08, 0, 1.0);

    // Lateral slip angle alpha with low-speed smoothing to prevent lateral jitter at standstill
    const rawAlpha = Math.atan2(vy, vLongRef);
    const lateralBlend = clamp(totalGroundSpeed / 0.08, 0, 1.0);

    return {
      kappa: clamp(rawKappa * longBlend, -1.5, 1.5),
      alpha: clamp(rawAlpha * lateralBlend, -Math.PI / 2, Math.PI / 2),
    };
  }
}
