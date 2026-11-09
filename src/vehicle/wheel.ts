/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import * as THREE from 'three';
import { clamp } from '../utils/math';
import { RigidBody } from './rigidBody';
import { WheelState } from './suspension';
import { TireModel, SurfaceType, TireEvaluationResult, SURFACE_PACEJKA } from './tire';

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

  // Stiction state for handbrake & slope hold
  public isStuck: boolean = false;

  // Stage A8: Soft-soil sinkage & bulldozing telemetry
  public sinkageDepth: number = 0;     // Soil sinkage depth z_s in meters (0 to maxSinkage)
  public bulldozingForce: number = 0;  // Bulldozing drag resistance force F_b in Newtons

  // Tire force results
  public Fx: number = 0;               // Longitudinal tractive/braking force (N)
  public Fy: number = 0;               // Lateral cornering force (N)
  public slipRatio: number = 0;        // Kappa [-1 .. +1]
  public slipAngleDeg: number = 0;     // Alpha (deg)
  public slipAngleRad: number = 0;     // Alpha (rad)
  public gripUtilization: number = 0;  // 0 .. 1.0 (friction ellipse radius)
  public surface: SurfaceType = 'asphalt';

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
    this.isStuck = false;
    this.sinkageDepth = 0;
    this.bulldozingForce = 0;
    this.Fx = 0;
    this.Fy = 0;
    this.slipRatio = 0;
    this.slipAngleDeg = 0;
    this.slipAngleRad = 0;
    this.gripUtilization = 0;
  }

  /**
   * Evaluates contact patch kinematics, tire forces, soft-soil sinkage, bulldozing drag,
   * and integrates wheel rotational dynamics.
   */
  public update(
    suspensionWheel: WheelState,
    rigidBody: RigidBody,
    driveTorque: number,
    brakeTorque: number,
    surfaceType: SurfaceType,
    dt: number,
    isHandbrakeActive: boolean = false
  ): void {
    this.surface = surfaceType;
    suspensionWheel.steerAngle = this.steerAngle;

    // =========================================================================
    // AIRBORNE WHEEL DYNAMICS
    // =========================================================================
    if (!suspensionWheel.grounded || suspensionWheel.normalLoad <= 1.0) {
      this.isStuck = false;
      this.sinkageDepth = 0;
      this.bulldozingForce = 0;
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
    this._groundForward.copy(this._bodyForward).addScaledVector(normal, -dotForwardNormal);
    if (this._groundForward.lengthSq() > 0.0001) {
      this._groundForward.normalize();
    } else {
      this._groundForward.copy(this._bodyForward);
    }

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
    const contactSpeedKmh = Math.abs(vx) * 3.6;

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
    // HANDBRAKE STICTION & SLOPE HOLD LOGIC
    // =========================================================================
    const isHandbrake = isHandbrakeActive || (brakeTorque >= 2000 && (this.id === 'RL' || this.id === 'RR'));
    const isRearWheel = this.id === 'RL' || this.id === 'RR';

    // Surface Pacejka & Static friction coefficient
    const coeffs = SURFACE_PACEJKA[surfaceType] ?? SURFACE_PACEJKA.dirt;
    const muStatic = coeffs.muStatic;

    // Maximum static grip capability on slopes
    const isSteepSlope = Math.abs(this._uLong.y) > 0.15;
    const minSlopeHoldFz = isSteepSlope ? 5500 : 0;
    const maxStaticGrip = Math.max(muStatic * 1.30 * suspensionWheel.normalLoad, minSlopeHoldFz);
    const maxStaticTorque = maxStaticGrip * this.radius;

    // A. Transition into STICK state:
    if (isHandbrake && isRearWheel) {
      if (!this.isStuck) {
        if (Math.abs(this.omega) < 0.5 || Math.abs(omegaFree) < 0.5 || contactSpeedKmh < 0.5) {
          this.isStuck = true;
        }
      }
    } else {
      this.isStuck = false;
    }

    // C. High-Torque Engine Breakaway:
    if (this.isStuck) {
      const driveTorqueMag = Math.abs(driveTorque);
      if (driveTorqueMag > maxStaticTorque && driveTorqueMag > brakeTorque) {
        this.isStuck = false;
      }
    }

    // =========================================================================
    // STAGE A8: SOFT-SOIL SINKAGE & BULLDOZING PHYSICS (WET MUD & SAND)
    // =========================================================================
    let kSoil = 0;
    let zMax = 0;
    let soilDensity = 0;
    let cBulldoze = 0;

    if (surfaceType === 'mud') {
      kSoil = 38000;
      zMax = 0.12;      // Wet Mud: high sinkage depth coefficient, max sinkage cap = 0.12m (~1/3 tire radius)
      soilDensity = 1800;
      cBulldoze = 180;
    } else if (surfaceType === 'sand') {
      kSoil = 56000;
      zMax = 0.08;      // Sand: medium sinkage depth coefficient, max sinkage cap = 0.08m
      soilDensity = 1600;
      cBulldoze = 150;
    }

    if (zMax > 0 && suspensionWheel.grounded && suspensionWheel.normalLoad > 10.0) {
      const normalLoad = suspensionWheel.normalLoad;
      const speed = Math.abs(vx);
      // Planing factor: sinkage decreases as vehicle speed increases
      const planingFactor = 1.0 / (1.0 + speed * 0.15);
      const staticSinkage = Math.min(zMax, (normalLoad / kSoil) * planingFactor);

      // Dynamic Digging / Wheelspin Effect:
      // Excessive wheelspin at low vehicle speeds causes tires to dig deep into soft soil ruts
      let targetSinkage = staticSinkage;
      const slipMag = Math.abs(this.slipRatio);
      if (slipMag > 0.15 && speed < 3.5) {
        const spinExcess = clamp((slipMag - 0.15) / 0.85, 0.0, 1.0);
        const lowSpeedWeight = clamp((3.5 - speed) / 3.5, 0.0, 1.0);
        const digFactor = spinExcess * lowSpeedWeight;
        targetSinkage = Math.min(zMax, staticSinkage + digFactor * (zMax - staticSinkage));
      }

      // Smooth sinkage tracking
      this.sinkageDepth = THREE.MathUtils.lerp(this.sinkageDepth, targetSinkage, Math.min(1.0, 12.0 * dt));

      // Bulldozing drag resistance force: F_b = c_bulldoze * (z_s^2) * tire_width * soil_density
      const tireWidth = 0.24; // 240mm tire width
      const zsSq = this.sinkageDepth * this.sinkageDepth;
      this.bulldozingForce = cBulldoze * zsSq * tireWidth * soilDensity;

      // Shift dynamic wheel contact center down by z_s:
      // Visibly and physically lowers tire mesh and contact patch into the terrain
      suspensionWheel.wheelWorldPosition.y -= this.sinkageDepth;
      suspensionWheel.contactPoint.y -= this.sinkageDepth;
    } else {
      // Non-Deformable Surface Bypass: Asphalt, Rock, Ice, Dry Dirt maintain zero sinkage (z_s = 0.00 m)
      this.sinkageDepth = 0.0;
      this.bulldozingForce = 0.0;
    }

    // =========================================================================
    // STICK STATE CONSTRAINT SOLVER
    // =========================================================================
    if (this.isStuck) {
      this.omega = 0;
      this.rpm = 0;
      this.slipRatio = 0;
      this.slipAngleDeg = 0;
      this.slipAngleRad = 0;

      const fGravityLongHold = 0.5 * rigidBody.mass * rigidBody.gravity * this._uLong.y;
      const fArrestLong = -clamp(25000 * vx, -4000, 4000);
      const fDriveLong = driveTorque / this.radius;

      const targetFx = fGravityLongHold + fArrestLong - fDriveLong;

      const fGravityLatHold = 0.5 * rigidBody.mass * rigidBody.gravity * this._uLat.y;
      const fArrestLat = -clamp(20000 * vy, -3000, 3000);
      const targetFy = fGravityLatHold + fArrestLat;

      const totalHoldForce = Math.hypot(targetFx, targetFy);
      if (totalHoldForce > maxStaticGrip) {
        const scale = maxStaticGrip / totalHoldForce;
        this.Fx = targetFx * scale;
        this.Fy = targetFy * scale;
        this.gripUtilization = 1.0;
      } else {
        this.Fx = targetFx;
        this.Fy = targetFy;
        this.gripUtilization = totalHoldForce / Math.max(1.0, maxStaticGrip);
      }

      const linSpeed = rigidBody.linearVelocity.length();
      if (linSpeed < 0.20) {
        if (linSpeed < 0.05) {
          rigidBody.linearVelocity.set(0, 0, 0);
          rigidBody.angularVelocity.set(0, 0, 0);
        } else {
          rigidBody.linearVelocity.multiplyScalar(0.70);
          rigidBody.angularVelocity.multiplyScalar(0.70);
        }
      }

      this._totalTireForce.copy(this._uLong)
        .multiplyScalar(this.Fx)
        .addScaledVector(this._uLat, this.Fy);

      rigidBody.applyForceAtWorldPoint(this._totalTireForce, suspensionWheel.contactPoint);
      return;
    }

    // =========================================================================
    // 2. SLIP KINEMATICS & PACEJKA EVALUATION (DYNAMIC REGIME)
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
      const syncReactionTorque = (slipDelta * this.inertia) / dt;
      const maxReactionFx = syncReactionTorque / this.radius;

      if (brakeLocked && Math.abs(driveTorque - finalFx * this.radius) <= brakeTorque) {
        omegaNext = 0;
      } else if (Math.abs(finalFx) >= Math.abs(maxReactionFx)) {
        omegaNext = omegaRoad;
        finalFx = maxReactionFx;
      } else {
        omegaNext = omegaFree - (finalFx * this.radius * this.invInertia) * dt;
      }
    } else {
      omegaNext = brakeLocked ? 0 : omegaRoad;
      finalFx = 0;
    }

    // Dynamic Rolling Resistance (F_rr = C_rr * N) and Bulldozing Drag (F_b) oppose vehicle motion
    if (Math.abs(vx) > 0.05) {
      finalFx -= Math.sign(vx) * (tireResult.rollingResForce + this.bulldozingForce);
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

    // Dynamic Rolling Resistance & Bulldozing drag torque on wheel (opposes wheel rotation)
    if (Math.abs(omegaNext) > 0.0001) {
      const totalDragTorque = (tireResult.rollingResForce + this.bulldozingForce) * this.radius;
      const stopDragTorque = (Math.abs(omegaNext) * this.inertia) / dt;
      if (totalDragTorque >= stopDragTorque) {
        omegaNext = 0;
      } else {
        omegaNext -= Math.sign(omegaNext) * (totalDragTorque * this.invInertia) * dt;
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
