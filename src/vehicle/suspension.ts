/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import * as THREE from 'three';
import { SCRATCH_VEC3_A, SCRATCH_VEC3_B, SCRATCH_VEC3_C, clamp } from '../utils/math';
import { RigidBody } from './rigidBody';
import { TestGround, GroundQuery } from '../world/testGround';

export interface WheelMountConfig {
  id: 'FL' | 'FR' | 'RL' | 'RR';
  localMountPoint: THREE.Vector3; // Top of suspension strut in body space
  restLength: number;            // Full extension length down from mount (m)
  maxTravel: number;             // Suspension travel stroke (m) (0.35m)
  wheelRadius: number;           // Wheel outer radius (m) (0.38m)
  springStiffness: number;       // N/m (e.g. 26000)
  preloadForce: number;          // N (per-wheel static preload)
  bumpDamping: number;           // N*s/m (compression damping)
  reboundDamping: number;        // N*s/m (extension damping)
}

export class WheelState {
  public readonly id: 'FL' | 'FR' | 'RL' | 'RR';
  public readonly config: WheelMountConfig;

  // Real-time wheel telemetry exposed for debug & rendering
  public compression: number = 0;              // Current spring compression (m, 0 to maxTravel)
  public compressionVelocity: number = 0;      // Rate of spring compression change (m/s)
  public normalLoad: number = 0;               // Normal force Fz (N)
  public grounded: boolean = false;            // Ground contact flag
  public wasGrounded: boolean = false;         // PART C.1: Track previous ground state to prevent first-contact spike
  public readonly contactPoint = new THREE.Vector3();
  public readonly contactNormal = new THREE.Vector3(0, 1, 0);
  public readonly wheelWorldPosition = new THREE.Vector3();

  // Internal history
  public previousCompression: number = 0;

  constructor(config: WheelMountConfig) {
    this.id = config.id;
    this.config = config;
  }
}

export interface SuspensionSystemConfig {
  frontArbStiffness: number; // Front anti-roll bar stiffness (N/m)
  rearArbStiffness: number;  // Rear anti-roll bar stiffness (N/m)
  bumpStopThreshold: number; // Fraction of stroke where bump stop engages (e.g. 0.85 = 85%)
  bumpStopStiffness: number; // Non-linear bump stop stiffness (N/m^2)
}

export class SuspensionSystem {
  public readonly wheels: WheelState[];
  public readonly wheelFL: WheelState;
  public readonly wheelFR: WheelState;
  public readonly wheelRL: WheelState;
  public readonly wheelRR: WheelState;

  // Anti-roll bars and progressive bump stop configuration
  public frontArbStiffness: number;
  public rearArbStiffness: number;
  public bumpStopThreshold: number;
  public bumpStopStiffness: number;

  constructor(config: Partial<SuspensionSystemConfig> = {}) {
    this.frontArbStiffness = config.frontArbStiffness ?? 9500;
    this.rearArbStiffness = config.rearArbStiffness ?? 7500;
    this.bumpStopThreshold = config.bumpStopThreshold ?? 0.80;
    this.bumpStopStiffness = config.bumpStopStiffness ?? 150000;

    const restLength = 0.65;
    const maxTravel = 0.35;
    const wheelRadius = 0.38;

    const springK = 26000;
    const preload = 1295;
    const bumpC = 3600;
    const reboundC = 5200;

    // PART A.4: Swap the x sign of the four wheel mounts so labels are physically correct:
    // Left side is +X (FL = +0.80, RL = +0.80), Right side is -X (FR = -0.80, RR = -0.80)
    const flConfig: WheelMountConfig = {
      id: 'FL',
      localMountPoint: new THREE.Vector3(0.80, -0.15, 1.30),
      restLength,
      maxTravel,
      wheelRadius,
      springStiffness: springK,
      preloadForce: preload,
      bumpDamping: bumpC,
      reboundDamping: reboundC,
    };

    const frConfig: WheelMountConfig = {
      id: 'FR',
      localMountPoint: new THREE.Vector3(-0.80, -0.15, 1.30),
      restLength,
      maxTravel,
      wheelRadius,
      springStiffness: springK,
      preloadForce: preload,
      bumpDamping: bumpC,
      reboundDamping: reboundC,
    };

    const rlConfig: WheelMountConfig = {
      id: 'RL',
      localMountPoint: new THREE.Vector3(0.80, -0.15, -1.30),
      restLength,
      maxTravel,
      wheelRadius,
      springStiffness: springK,
      preloadForce: preload,
      bumpDamping: bumpC,
      reboundDamping: reboundC,
    };

    const rrConfig: WheelMountConfig = {
      id: 'RR',
      localMountPoint: new THREE.Vector3(-0.80, -0.15, -1.30),
      restLength,
      maxTravel,
      wheelRadius,
      springStiffness: springK,
      preloadForce: preload,
      bumpDamping: bumpC,
      reboundDamping: reboundC,
    };

    this.wheelFL = new WheelState(flConfig);
    this.wheelFR = new WheelState(frConfig);
    this.wheelRL = new WheelState(rlConfig);
    this.wheelRR = new WheelState(rrConfig);

    this.wheels = [this.wheelFL, this.wheelFR, this.wheelRL, this.wheelRR];
  }

  /**
   * Evaluates raycasts, spring forces, damping, anti-roll bars, and normal suspension load.
   */
  public update(rigidBody: RigidBody, world: TestGround, dt: number): void {
    // 1. Raycast each wheel down to find compression and contact points
    for (let i = 0; i < 4; i++) {
      const wheel = this.wheels[i];
      const cfg = wheel.config;

      // Mount point in world space
      const worldMount = SCRATCH_VEC3_A;
      rigidBody.localToWorld(cfg.localMountPoint, worldMount);

      // Suspension raycast downwards: ray along vehicle's local down axis
      const bodyDown = SCRATCH_VEC3_B.set(0, -1, 0).applyQuaternion(rigidBody.quaternion).normalize();

      // Query ground directly below mount position
      const groundInfo: GroundQuery = world.getHeightNormalSurface(worldMount.x, worldMount.z);

      const groundY = groundInfo.height;
      const verticalDist = worldMount.y - groundY;
      const maxReach = cfg.restLength + cfg.wheelRadius;

      if (verticalDist < maxReach && verticalDist > -0.5) {
        wheel.grounded = true;
        const unconstrainedCompression = maxReach - verticalDist;
        wheel.compression = clamp(unconstrainedCompression, 0, cfg.maxTravel);

        // PART C.1: First contact spike suppression
        if (!wheel.wasGrounded) {
          wheel.previousCompression = wheel.compression;
        }

        // PART C.1: Clamp compressionVelocity to -5 .. +5 m/s
        const rawCompVel = (wheel.compression - wheel.previousCompression) / Math.max(0.0001, dt);
        wheel.compressionVelocity = clamp(rawCompVel, -5.0, 5.0);
        wheel.previousCompression = wheel.compression;

        // Contact point on ground
        wheel.contactPoint.set(worldMount.x, groundY, worldMount.z);
        wheel.contactNormal.copy(groundInfo.normal);

        // PART B: Compute wheelWorldPosition from mount and clamped compression (never pops above body)
        wheel.wheelWorldPosition.copy(worldMount).addScaledVector(bodyDown, cfg.restLength - wheel.compression);

        wheel.wasGrounded = true;
      } else {
        // Airborne branch
        wheel.grounded = false;
        wheel.wasGrounded = false;
        wheel.compression = 0;
        wheel.compressionVelocity = 0;
        wheel.previousCompression = 0;
        wheel.normalLoad = 0;
        wheel.wheelWorldPosition.copy(worldMount).addScaledVector(bodyDown, cfg.restLength);
      }
    }

    // 2. Anti-Roll Bar (ARB) load transfer
    const frontArbForce = (this.wheelFL.compression - this.wheelFR.compression) * this.frontArbStiffness;
    const rearArbForce = (this.wheelRL.compression - this.wheelRR.compression) * this.rearArbStiffness;

    // 3. Compute and apply normal suspension forces at each wheel contact
    for (let i = 0; i < 4; i++) {
      const wheel = this.wheels[i];
      if (!wheel.grounded) {
        wheel.normalLoad = 0;
        continue;
      }

      const cfg = wheel.config;

      // Base spring force: F = K * x + Preload
      let springForce = cfg.springStiffness * wheel.compression + cfg.preloadForce;

      // Damping force: separate bump and rebound damping
      let dampingForce = 0;
      if (wheel.compressionVelocity > 0) {
        dampingForce = cfg.bumpDamping * wheel.compressionVelocity;
      } else {
        dampingForce = cfg.reboundDamping * wheel.compressionVelocity;
      }

      // Progressive bump stop near full travel stroke
      const bumpStopEngagement = cfg.maxTravel * this.bumpStopThreshold;
      let bumpStopForce = 0;
      if (wheel.compression > bumpStopEngagement) {
        const excess = wheel.compression - bumpStopEngagement;
        bumpStopForce = this.bumpStopStiffness * (excess ** 2);
      }

      // Anti-roll bar adjustment
      let arbDelta = 0;
      if (wheel.id === 'FL') arbDelta = -frontArbForce;
      else if (wheel.id === 'FR') arbDelta = frontArbForce;
      else if (wheel.id === 'RL') arbDelta = -rearArbForce;
      else if (wheel.id === 'RR') arbDelta = rearArbForce;

      const rawLoad = springForce + dampingForce + bumpStopForce + arbDelta;
      wheel.normalLoad = Math.max(0, rawLoad);

      // Suspension force vector: push along contact surface normal
      const forceVec = SCRATCH_VEC3_C.copy(wheel.contactNormal).multiplyScalar(wheel.normalLoad);
      rigidBody.applyForceAtWorldPoint(forceVec, wheel.contactPoint);
    }
  }
}
