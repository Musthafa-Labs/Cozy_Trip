/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import { clamp, lerp } from '../utils/math';

export interface TorqueCurvePoint {
  rpm: number;
  torque: number; // N*m at wide-open throttle (WOT)
}

export interface EngineConfig {
  idleRpm: number;           // Idle speed in RPM (default: 900)
  peakTorqueRpm: number;     // RPM of peak torque (default: 4500)
  redlineRpm: number;        // Maximum allowed RPM (default: 7000)
  revLimiterCutRpm: number;  // Hard cut threshold (default: 7000)
  revLimiterResumeRpm: number; // Threshold to resume fuel injection (default: 6850)
  inertia: number;           // Engine flywheel + rotating assembly inertia in kg*m^2 (default: 0.20)
  throttleResponseTau: number; // 1st-order throttle response filter time constant in seconds (default: 0.08)
  engineBrakingBase: number; // Base engine friction braking in N*m (default: 18.0)
  engineBrakingRpmFactor: number; // Speed-dependent engine braking factor N*m/RPM (default: 0.016)
  torqueCurve: TorqueCurvePoint[]; // Piecewise torque curve points
}

// Vintage 4.0L Inline-6 / V8 off-road engine profile
export const DEFAULT_TORQUE_CURVE: TorqueCurvePoint[] = [
  { rpm: 0, torque: 180 },
  { rpm: 900, torque: 240 },
  { rpm: 1800, torque: 320 },
  { rpm: 3000, torque: 385 },
  { rpm: 4500, torque: 420 }, // Peak torque ~4500 RPM
  { rpm: 5500, torque: 380 },
  { rpm: 6500, torque: 310 },
  { rpm: 7000, torque: 260 }, // Redline 7000 RPM
  { rpm: 7500, torque: 180 },
];

export class Engine {
  public readonly config: EngineConfig;

  // State
  public rpm: number = 900;
  public omega: number = (900 * 2 * Math.PI) / 60; // rad/s
  public throttle: number = 0;                     // Smoothed throttle [0 .. 1.0]
  public outputTorque: number = 0;                 // Net output torque at flywheel (N*m)
  public rawCombustionTorque: number = 0;          // Gross combustion torque (N*m)
  public engineBrakingTorque: number = 0;          // Retarding torque (N*m)
  public isRevLimiterActive: boolean = false;

  constructor(config: Partial<EngineConfig> = {}) {
    this.config = {
      idleRpm: config.idleRpm ?? 900,
      peakTorqueRpm: config.peakTorqueRpm ?? 4500,
      redlineRpm: config.redlineRpm ?? 7000,
      revLimiterCutRpm: config.revLimiterCutRpm ?? 7000,
      revLimiterResumeRpm: config.revLimiterResumeRpm ?? 6850,
      inertia: config.inertia ?? 0.20,
      throttleResponseTau: config.throttleResponseTau ?? 0.08,
      engineBrakingBase: config.engineBrakingBase ?? 18.0,
      engineBrakingRpmFactor: config.engineBrakingRpmFactor ?? 0.016,
      torqueCurve: config.torqueCurve ?? DEFAULT_TORQUE_CURVE,
    };

    this.rpm = this.config.idleRpm;
    this.omega = (this.rpm * 2 * Math.PI) / 60;
  }

  public reset(): void {
    this.rpm = this.config.idleRpm;
    this.omega = (this.rpm * 2 * Math.PI) / 60;
    this.throttle = 0;
    this.outputTorque = 0;
    this.rawCombustionTorque = 0;
    this.engineBrakingTorque = 0;
    this.isRevLimiterActive = false;
  }

  /**
   * Interpolates the engine wide-open throttle (WOT) torque from the torque curve table.
   */
  public getWOTTorque(rpm: number): number {
    const curve = this.config.torqueCurve;
    if (rpm <= curve[0].rpm) return curve[0].torque;
    const last = curve[curve.length - 1];
    if (rpm >= last.rpm) return last.torque;

    for (let i = 0; i < curve.length - 1; i++) {
      const p0 = curve[i];
      const p1 = curve[i + 1];
      if (rpm >= p0.rpm && rpm <= p1.rpm) {
        const factor = (rpm - p0.rpm) / (p1.rpm - p0.rpm);
        return lerp(p0.torque, p1.torque, factor);
      }
    }
    return curve[0].torque;
  }

  /**
   * Updates engine RPM and output torque given throttle pedal input and clutch reaction load.
   *
   * @param throttleInput Player throttle pedal position [0 .. 1.0]
   * @param clutchReactionTorque Reaction torque transmitted through clutch (N*m)
   * @param clutchEngagement Clutch lockup factor [0 = disengaged, 1.0 = locked]
   * @param targetDrivetrainRpm Wheel-coupled RPM from driveshaft
   * @param dt Fixed physics timestep in seconds (1/120)
   */
  public update(
    throttleInput: number,
    clutchReactionTorque: number,
    clutchEngagement: number,
    targetDrivetrainRpm: number,
    dt: number
  ): void {
    // 1. Throttle input filtering (smooth 1st-order response)
    const clampedInput = clamp(throttleInput, 0, 1.0);
    const alpha = clamp(dt / (this.config.throttleResponseTau + dt), 0.05, 1.0);
    this.throttle = lerp(this.throttle, clampedInput, alpha);

    // 2. Rev limiter state logic
    if (this.rpm >= this.config.revLimiterCutRpm) {
      this.isRevLimiterActive = true;
    } else if (this.rpm <= this.config.revLimiterResumeRpm) {
      this.isRevLimiterActive = false;
    }

    // 3. Wide-open-throttle torque & net combustion torque
    const wotTorque = this.getWOTTorque(this.rpm);
    let combustionTorque = wotTorque * this.throttle;

    // Fuel cut during rev limiter activation
    if (this.isRevLimiterActive) {
      combustionTorque = 0;
    }

    // 4. Closed-loop Idle Governor
    // When idling at zero throttle, inject minimal torque to sustain idle speed
    if (this.throttle < 0.01 && this.rpm <= this.config.idleRpm + 50) {
      const idleDeficit = Math.max(0, this.config.idleRpm - this.rpm);
      const idleGovernorTorque = clamp(idleDeficit * 0.85, 0, 60.0);
      combustionTorque = Math.max(combustionTorque, idleGovernorTorque);
    }

    this.rawCombustionTorque = combustionTorque;

    // 5. Engine friction & engine braking torque (always opposes rotation)
    this.engineBrakingTorque = this.config.engineBrakingBase + this.config.engineBrakingRpmFactor * this.rpm;

    // 6. Net flywheel torque before clutch
    // When throttle is released, net torque is negative (engine braking)
    const netFlywheelTorque = combustionTorque - this.engineBrakingTorque;

    // 7. Dynamic Engine RPM Integration
    if (clutchEngagement >= 0.95) {
      // Locked clutch regime: Engine RPM is rigidly coupled to driveshaft rotation
      const coupledRpm = Math.max(this.config.idleRpm, targetDrivetrainRpm);
      this.rpm = clamp(coupledRpm, this.config.idleRpm, this.config.redlineRpm + 100);
      this.omega = (this.rpm * 2 * Math.PI) / 60;

      // In locked regime, engine delivers net torque directly to the clutch
      this.outputTorque = netFlywheelTorque;
    } else {
      // Slipping clutch or neutral/idle regime:
      // Torque transmitted across clutch scales with slip speed and clutch engagement
      const slipRpm = this.rpm - targetDrivetrainRpm;
      const normalizedSlip = clamp(slipRpm / 400, -1.0, 1.0);
      const clutchSlipTorque = normalizedSlip * 480.0 * clutchEngagement;

      const netTorqueOnFlywheel = netFlywheelTorque - clutchSlipTorque;
      const dOmega = (netTorqueOnFlywheel / this.config.inertia) * dt;
      this.omega += dOmega;

      // Prevent engine from stalling below idle speed (automatic idle governor catch)
      const minOmega = (this.config.idleRpm * 2 * Math.PI) / 60;
      if (this.omega < minOmega) {
        this.omega = minOmega;
      }

      // Cap at rev limiter
      const maxOmega = (this.config.redlineRpm * 2 * Math.PI) / 60;
      if (this.omega > maxOmega) {
        this.omega = maxOmega;
      }

      this.rpm = (this.omega * 60) / (2 * Math.PI);

      // Output torque transmitted into transmission is the clutch slip torque
      this.outputTorque = clutchSlipTorque;
    }

    // Safety: at zero throttle and vehicle standstill, output torque must not creep
    if (this.throttle < 0.005 && clutchEngagement < 0.1) {
      this.outputTorque = 0;
    }
  }
}
