/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import * as THREE from 'three';
import { clamp } from '../utils/math';
import { RigidBody } from './rigidBody';
import { WheelState } from './suspension';
import { TireModel, SurfaceType, TireEvaluationResult } from './tire';

export interface WheelDynamicsConfig {
  inertia: number;      // kg*m^2 (approx 1.2 kg*m^2 for wheel + 30" off-road tire)
  radius: number;       // m (0.38 m)
}

export class WheelDynamics {
  public readonly id: 'FL' | 'FR' | 'RL' | 'RR';
  public readonly inertia: number;
  public readonly invInertia: number;
  public readonly radius: number;

  // Rotational state
  public omega: number = 0;        // Angular velocity (rad/s)
  public spinAngle: number = 0;    // Accumulated rotation angle for 3D wheel visual (rad)
  public steerAngle: number = 0;   // Current steer angle (rad, positive = LEFT)
  public rpm: number = 0;          // Wheel RPM for debug HUD

  // Tire force results
  public Fx: number = 0;               // Longitudinal tractive/braking force (N)
  public Fy: number = 0;               // Lateral cornering force (N)
  public slipRatio: number = 0;        // Kappa [-1 .. +1]
  public slipAngleDeg: number = 0;     // Alpha (deg)
  public slipAngleRad: number = 0;     // Alpha (rad)
  public gripUtilization: number = 0;  // 0 .. 1.0 (friction ellipse radius)
  public surface: SurfaceType = 'dirt';

  // Vectors for visual force arrows in debug visualizer
  public readonly worldLongitudinalDir = new THREE.Vector3();
  public readonly worldLateralDir = new THREE.Vector3();

  // DEDICATED private instance vectors to guarantee ZERO allocation and ZERO scratch pollution
  private readonly _bodyForward = new THREE.Vector3();
  private readonly _bodyRight = new THREE.Vector3();
  private readonly _groundForward = new THREE.Vector3();
  private readonly _uLong = new THREE.Vector3();
  private readonly _uLat = new THREE.Vector3();
  private readonly _contactVel = new THREE.Vector3();
  private readonly _totalTireForce = new THREE.Vector3();
  private readonly _alignTorque = new THREE.Vector3();

  constructor(id: 'FL' | 'FR' | 'RL' | 'RR', config: Partial<WheelDynamicsConfig> = {}) {
    this.id = id;
    this.inertia = config.inertia ?? 1.2;
    this.invInertia = 1.0 / this.inertia;
    this.radius = config.radius ?? 0.38;
  }

  public reset(): void {
    this.omega = 0;
    this.spinAngle = 0;
    this.steerAngle = 0;
    this.rpm = 0;
    this.Fx = 0;
    this.Fy = 0;
    this.slipRatio = 0;
    this.slipAngleDeg = 0;
    this.slipAngleRad = 0;
    this.gripUtilization = 0;
  }

  /**
   * Evaluates contact patch kinematics, tire forces, and integrates wheel rotational dynamics.
   */
  public update(
    suspensionWheel: WheelState,
    rigidBody: RigidBody,
    driveTorque: number,
    brakeTorque: number,
    surfaceType: SurfaceType,
    dt: number
  ): void {
    this.surface = surfaceType;

    // =========================================================================
    // AIRBORNE WHEEL DYNAMICS
    // =========================================================================
    if (!suspensionWheel.grounded || suspensionWheel.normalLoad <= 1.0) {
      let omegaNext = this.omega + (driveTorque * this.invInertia) * dt;

      // Brake torque deceleration
      if (brakeTorque > 0 && Math.abs(omegaNext) > 0.0001) {
        const stopTorque = (Math.abs(omegaNext) * this.inertia) / dt;
        if (brakeTorque >= stopTorque) {
          omegaNext = 0;
        } else {
          omegaNext -= Math.sign(omegaNext) * (brakeTorque * this.invInertia) * dt;
        }
      }

      // Bearing drag opposes airborne free spin without reversing
      if (Math.abs(omegaNext) > 0.0001) {
        const bearingTorque = 1.5 * omegaNext;
        const stopBearing = (Math.abs(omegaNext) * this.inertia) / dt;
        if (Math.abs(bearingTorque) >= stopBearing) {
          omegaNext = 0;
        } else {
          omegaNext -= (bearingTorque * this.invInertia) * dt;
        }
      }

      this.omega = omegaNext;
      if (Math.abs(this.omega) > 0.0001) {
        this.spinAngle += this.omega * dt;
      }
      this.rpm = (this.omega * 60) / (2 * Math.PI);
      this.Fx = 0;
      this.Fy = 0;
      this.slipRatio = 0;
      this.slipAngleDeg = 0;
      this.slipAngleRad = 0;
      this.gripUtilization = 0;
      return;
    }

    // =========================================================================
    // WHEEL FRAME AXES (SINGLE SOURCE OF TRUTH CONVENTION)
    // =========================================================================
    this._bodyForward.set(0, 0, 1).applyQuaternion(rigidBody.quaternion);
    this._bodyRight.set(1, 0, 0).applyQuaternion(rigidBody.quaternion);

    const normal = suspensionWheel.contactNormal;

    // Project forward onto ground plane
    const dotForwardNormal = this._bodyForward.dot(normal);
    this._groundForward.copy(this._bodyForward).addScaledVector(normal, -dotForwardNormal).normalize();

    // Rotate about ground normal by steerAngle
    if (Math.abs(this.steerAngle) > 0.0001) {
      this._uLong.copy(this._groundForward).applyAxisAngle(normal, this.steerAngle).normalize();
    } else {
      this._uLong.copy(this._groundForward);
    }

    // Lateral axis: with forward +Z, uLat = normal x uLong points to the car's LEFT (+X)
    this._uLat.crossVectors(normal, this._uLong).normalize();

    this.worldLongitudinalDir.copy(this._uLong);
    this.worldLateralDir.copy(this._uLat);

    // =========================================================================
    // CONTACT VELOCITY IN WORLD SPACE
    // v_contact = v_body + omega_body x (r_contact - r_com)
    // =========================================================================
    rigidBody.getPointVelocity(suspensionWheel.contactPoint, this._contactVel);

    const vx = this._contactVel.dot(this._uLong); // Longitudinal contact speed
    const vy = this._contactVel.dot(this._uLat);  // Lateral contact slip speed
    const omegaRoad = vx / this.radius;           // Angular velocity matching road speed

    // =========================================================================
    // 1. FREE-WHEEL INTEGRATION (Drive & Brake Torques)
    // =========================================================================
    let omegaFree = this.omega + (driveTorque * this.invInertia) * dt;

    // Service / handbrake torque (exact non-reversing solver)
    if (brakeTorque > 0 && Math.abs(omegaFree) > 0.0001) {
      const stopTorque = (Math.abs(omegaFree) * this.inertia) / dt;
      if (brakeTorque >= stopTorque) {
        omegaFree = 0;
      } else {
        omegaFree -= Math.sign(omegaFree) * (brakeTorque * this.invInertia) * dt;
      }
    }

    const brakeLocked = brakeTorque > 0 && omegaFree === 0;

    // Natural standstill stabilization when vehicle and wheel are at rest
    if (Math.abs(driveTorque) < 0.01 && Math.abs(vx) < 0.02 && Math.abs(omegaFree) < 0.02) {
      omegaFree = 0;
    }

    // =========================================================================
    // 2. SLIP KINEMATICS & PACEJKA EVALUATION
    // =========================================================================
    const { kappa, alpha } = TireModel.computeSlip(vx, vy, omegaFree, this.radius);
    this.slipRatio = kappa;
    this.slipAngleRad = alpha;
    this.slipAngleDeg = (alpha * 180) / Math.PI;

    const tireResult: TireEvaluationResult = TireModel.evaluate(
      kappa,
      alpha,
      suspensionWheel.normalLoad,
      surfaceType
    );

    // Lateral force with single-step speed reversal cap
    this.Fy = tireResult.Fy;
    const maxFy = (suspensionWheel.normalLoad / 9.81) * Math.abs(vy) / dt;
    this.Fy = clamp(this.Fy, -maxFy, maxFy);

    // =========================================================================
    // 3. TIRE-GROUND COUPLING (Adherent vs Sliding / Burnout Regimes)
    // =========================================================================
    let omegaNext = omegaFree;
    let finalFx = tireResult.Fx;

    const slipDelta = omegaFree - omegaRoad;
    if (Math.abs(slipDelta) > 0.0001) {
      // Torque required to synchronize wheel angular velocity with road speed in dt
      const syncReactionTorque = (slipDelta * this.inertia) / dt;
      const maxReactionFx = syncReactionTorque / this.radius;

      if (Math.abs(finalFx) >= Math.abs(maxReactionFx)) {
        // Adherent (rolling) regime: tire grip is sufficient to roll with road
        omegaNext = omegaRoad;
        finalFx = maxReactionFx;
      } else {
        // Sliding regime: burnout wheelspin or brake lockup
        if (brakeLocked && Math.abs(driveTorque - finalFx * this.radius) <= brakeTorque) {
          omegaNext = 0;
        } else {
          omegaNext = omegaFree - (finalFx * this.radius * this.invInertia) * dt;
        }
      }
    } else {
      omegaNext = omegaRoad;
      finalFx = 0;
    }

    // Rolling resistance opposes vehicle motion
    if (Math.abs(vx) > 0.05) {
      finalFx -= Math.sign(vx) * tireResult.rollingResForce;
    }

    this.Fx = finalFx;
    this.gripUtilization = tireResult.gripUtilization;

    // Apply tire force at contact patch
    this._totalTireForce.copy(this._uLong)
      .multiplyScalar(this.Fx)
      .addScaledVector(this._uLat, this.Fy);

    rigidBody.applyForceAtWorldPoint(this._totalTireForce, suspensionWheel.contactPoint);

    // Self-aligning torque Mz
    if (Math.abs(tireResult.aligningTorque) > 0.01) {
      this._alignTorque.copy(normal).multiplyScalar(tireResult.aligningTorque);
      rigidBody.applyTorque(this._alignTorque);
    }

    // Rolling resistance torque on wheel
    if (Math.abs(omegaNext) > 0.0001) {
      const rrTorque = tireResult.rollingResForce * this.radius;
      const stopRRTorque = (Math.abs(omegaNext) * this.inertia) / dt;
      if (rrTorque >= stopRRTorque) {
        omegaNext = 0;
      } else {
        omegaNext -= Math.sign(omegaNext) * (rrTorque * this.invInertia) * dt;
      }
    }

    // Final standstill zero-lock
    if (Math.abs(omegaNext) < 0.0005 && Math.abs(driveTorque) < 0.01) {
      omegaNext = 0;
    }

    this.omega = omegaNext;

    // Update spinAngle ONLY when wheel is actually rotating
    if (Math.abs(this.omega) > 0.0001) {
      this.spinAngle += this.omega * dt;
    }
    this.rpm = (this.omega * 60) / (2 * Math.PI);
  }
}
