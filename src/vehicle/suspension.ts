/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import * as THREE from 'three';
import { clamp } from '../utils/math';
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

export interface WheelRayConfig {
  id: string;
  angleRad: number;           // Angle from strut down axis (rad, positive = forward, negative = rearward)
  longitudinalOffset: number; // Offset along wheel longitudinal axis (m)
  circOffset: number;         // Circular perimeter offset along strut axis: R * cos(angle) (m)
}

/**
 * 5-Ray Array per wheel spanning the lower contact arc:
 * - center (0°): directly below wheel axle, dominant on level ground and uniform slopes
 * - front_bottom (+22.5°): low-angle bumps and rising terrain
 * - leading_face (+45°): curbs, rocks, and step obstacles (~0.10 - 0.18m high)
 * - high_leading (+68°): sharp vertical lips and obstacles up to 0.25m high
 * - rear_bottom (-25°): trailing perimeter to prevent drop-off snagging on curbs and trenches
 */
export const WHEEL_RAYS: WheelRayConfig[] = [
  {
    id: 'center',
    angleRad: 0.0,
    longitudinalOffset: 0.0,
    circOffset: 0.38,
  },
  {
    id: 'front_bottom',
    angleRad: 0.39270, // 22.5°
    longitudinalOffset: 0.38 * Math.sin(0.39270), // ~0.145 m
    circOffset: 0.38 * Math.cos(0.39270),         // ~0.351 m
  },
  {
    id: 'leading_face',
    angleRad: 0.78540, // 45.0°
    longitudinalOffset: 0.38 * Math.sin(0.78540), // ~0.269 m
    circOffset: 0.38 * Math.cos(0.78540),         // ~0.269 m
  },
  {
    id: 'high_leading',
    angleRad: 1.18682, // 68.0°
    longitudinalOffset: 0.38 * Math.sin(1.18682), // ~0.352 m
    circOffset: 0.38 * Math.cos(1.18682),         // ~0.142 m
  },
  {
    id: 'rear_bottom',
    angleRad: -0.43633, // -25.0°
    longitudinalOffset: 0.38 * Math.sin(-0.43633), // ~ -0.161 m
    circOffset: 0.38 * Math.cos(-0.43633),         // ~0.344 m
  },
];

export class WheelState {
  public readonly id: 'FL' | 'FR' | 'RL' | 'RR';
  public readonly config: WheelMountConfig;

  // Real-time wheel telemetry exposed for debug & rendering
  public compression: number = 0;              // Current spring compression (m, 0 to maxTravel)
  public compressionVelocity: number = 0;      // Rate of spring compression change (m/s)
  public normalLoad: number = 0;               // Normal force Fz (N)
  public grounded: boolean = false;            // Ground contact flag
  public wasGrounded: boolean = false;         // Track previous ground state to prevent first-contact spike
  public readonly contactPoint = new THREE.Vector3();
  public readonly contactNormal = new THREE.Vector3(0, 1, 0);
  public readonly rawContactNormal = new THREE.Vector3(0, 1, 0); // Unsmoothed target normal
  public readonly wheelWorldPosition = new THREE.Vector3();
  public steerAngle: number = 0;               // Steer angle of this wheel (rad)
  public dominantRayId: string = 'center';     // Debug telemetry for active contact ray

  // Over-travel hard stop telemetry and state
  public overTravel: number = 0;               // Travel beyond maxTravel (m)
  public previousOverTravel: number = 0;       // Previous frame overTravel (m)
  public overTravelVelocity: number = 0;       // Rate of change of overTravel (m/s)

  // Internal history
  public previousCompression: number = 0;

  constructor(config: WheelMountConfig) {
    this.id = config.id;
    this.config = config;
  }
}

export interface SuspensionSystemConfig {
  frontArbStiffness?: number; // Front anti-roll bar stiffness (N/m)
  rearArbStiffness?: number;  // Rear anti-roll bar stiffness (N/m)
  bumpStopThreshold?: number; // Fraction of stroke where bump stop engages (e.g. 0.85 = 85%)
  bumpStopStiffness?: number; // Non-linear bump stop stiffness (N/m^2)
  hardStopStiffness?: number; // Hard stop stiffness beyond maxTravel (N/m)
  hardStopDamping?: number;   // Hard stop damping beyond maxTravel (N*s/m)
  hardStopMaxForce?: number;  // Hard stop maximum clamping force (N)
  normalSmoothingAlpha?: number; // Normal exponential smoothing factor at 120 Hz (0.35)
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

  // Hard stop configuration for suspension bottom-out
  public hardStopStiffness: number; // Hard stop stiffness beyond maxTravel (N/m)
  public hardStopDamping: number;   // Hard stop damping beyond maxTravel (N*s/m)
  public hardStopMaxForce: number;  // Hard stop maximum clamping force (N)

  // Contact normal smoothing coefficient at 120 Hz
  public normalSmoothingAlpha: number;

  // Pre-allocated dedicated scratch vectors to guarantee ZERO per-frame allocations
  private readonly _worldMount = new THREE.Vector3();
  private readonly _bodyDown = new THREE.Vector3();
  private readonly _bodyUp = new THREE.Vector3();
  private readonly _bodyFwd = new THREE.Vector3();
  private readonly _wheelFwd = new THREE.Vector3();
  private readonly _probePos = new THREE.Vector3();
  private readonly _groundPoint = new THREE.Vector3();
  private readonly _diff = new THREE.Vector3();
  private readonly _circleNormal = new THREE.Vector3();
  private readonly _targetNormal = new THREE.Vector3();
  private readonly _dominantContactPoint = new THREE.Vector3();
  private readonly _dominantGroundNormal = new THREE.Vector3();
  private readonly _forceVec = new THREE.Vector3();

  constructor(config: Partial<SuspensionSystemConfig> = {}) {
    this.frontArbStiffness = config.frontArbStiffness ?? 9500;
    this.rearArbStiffness = config.rearArbStiffness ?? 7500;
    this.bumpStopThreshold = config.bumpStopThreshold ?? 0.80;
    this.bumpStopStiffness = config.bumpStopStiffness ?? 150000;

    this.hardStopStiffness = config.hardStopStiffness ?? 200000; // N/m
    this.hardStopDamping = config.hardStopDamping ?? 40000;       // N*s/m
    this.hardStopMaxForce = config.hardStopMaxForce ?? 45000;    // N
    this.normalSmoothingAlpha = config.normalSmoothingAlpha ?? 0.35;

    const restLength = 0.65;
    const maxTravel = 0.35;
    const wheelRadius = 0.38;

    const springK = 26000;
    const preload = 1295;
    const bumpC = 3600;
    const reboundC = 5200;

    // Wheel mounts in body space (+X left, -X right, +Z forward, -Z rear):
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
   * Evaluates multi-ray ground contact, spring forces, damping, anti-roll bars,
   * leading-edge obstacle detection, and normal suspension load.
   */
  public update(rigidBody: RigidBody, world: TestGround, dt: number, steerAngles?: number[]): void {
    // 1. Multi-Ray Array contact detection per wheel
    for (let i = 0; i < 4; i++) {
      const wheel = this.wheels[i];
      const cfg = wheel.config;

      if (steerAngles && steerAngles[i] !== undefined) {
        wheel.steerAngle = steerAngles[i];
      }

      // Mount point in world space
      rigidBody.localToWorld(cfg.localMountPoint, this._worldMount);

      // Orientation axes in world space
      this._bodyDown.set(0, -1, 0).applyQuaternion(rigidBody.quaternion).normalize();
      this._bodyUp.copy(this._bodyDown).negate();
      this._bodyFwd.set(0, 0, 1).applyQuaternion(rigidBody.quaternion).normalize();

      // Steered wheel forward axis
      if (Math.abs(wheel.steerAngle) > 0.0001) {
        this._wheelFwd.copy(this._bodyFwd).applyAxisAngle(this._bodyUp, wheel.steerAngle).normalize();
      } else {
        this._wheelFwd.copy(this._bodyFwd);
      }

      // Query multi-ray array to find highest contact elevation / maximum required compression
      let maxComp = -Infinity;
      let dominantIdx = -1;
      let isGrounded = false;

      for (let k = 0; k < WHEEL_RAYS.length; k++) {
        const ray = WHEEL_RAYS[k];

        // Probe ground position along steered wheel plane
        this._probePos.copy(this._worldMount).addScaledVector(this._wheelFwd, ray.longitudinalOffset);
        const gInfo: GroundQuery = world.getHeightNormalSurface(this._probePos.x, this._probePos.z);
        this._groundPoint.set(this._probePos.x, gInfo.height, this._probePos.z);

        // Distance from strut mount to ground along the suspension downward travel axis
        const distDown = this._diff.subVectors(this._groundPoint, this._worldMount).dot(this._bodyDown);
        const reach = cfg.restLength + ray.circOffset;

        if (distDown < reach && distDown > -0.5) {
          isGrounded = true;
          const comp = reach - distDown;

          if (comp > maxComp) {
            maxComp = comp;
            dominantIdx = k;
            this._dominantContactPoint.copy(this._groundPoint);
            this._dominantGroundNormal.copy(gInfo.normal);
          }
        }
      }

      if (isGrounded && dominantIdx >= 0) {
        wheel.grounded = true;
        wheel.dominantRayId = WHEEL_RAYS[dominantIdx].id;

        const unconstrainedCompression = maxComp;
        wheel.compression = clamp(unconstrainedCompression, 0, cfg.maxTravel);

        // Bottom-out over-travel calculation
        const over = Math.max(0, unconstrainedCompression - cfg.maxTravel);
        if (!wheel.wasGrounded) {
          wheel.previousOverTravel = over;
        }
        wheel.overTravelVelocity = clamp((over - wheel.previousOverTravel) / Math.max(0.0001, dt), -5, 5);
        wheel.previousOverTravel = over;
        wheel.overTravel = over;

        // First contact spike suppression
        if (!wheel.wasGrounded) {
          wheel.previousCompression = wheel.compression;
        }

        // Clamp compressionVelocity to -5 .. +5 m/s to prevent numerical force shock
        const rawCompVel = (wheel.compression - wheel.previousCompression) / Math.max(0.0001, dt);
        wheel.compressionVelocity = clamp(rawCompVel, -5.0, 5.0);
        wheel.previousCompression = wheel.compression;

        // Compute contact normal from dominant ray geometry
        const domRay = WHEEL_RAYS[dominantIdx];

        // Circle normal pointing from the contact perimeter point inward toward the wheel hub
        this._circleNormal.copy(this._bodyUp).multiplyScalar(Math.cos(domRay.angleRad))
          .addScaledVector(this._wheelFwd, -Math.sin(domRay.angleRad)).normalize();

        // Blend circle contact normal with terrain surface normal based on ray angle:
        // On level ground and smooth slopes, domRay is center (angle = 0) -> 100% terrain normal
        // On sharp steps and obstacles, forward circle normal blends in to provide climbing reaction
        const blendWeight = Math.abs(Math.sin(domRay.angleRad));
        this._targetNormal.copy(this._dominantGroundNormal).lerp(this._circleNormal, blendWeight).normalize();
        wheel.rawContactNormal.copy(this._targetNormal);

        // Apply exponential smoothing at 120 Hz to prevent abrupt impulse spikes on sharp edges
        if (!wheel.wasGrounded) {
          wheel.contactNormal.copy(this._targetNormal);
        } else {
          wheel.contactNormal.lerp(this._targetNormal, this.normalSmoothingAlpha).normalize();
        }

        // Effective contact point on obstacle / terrain surface
        wheel.contactPoint.copy(this._dominantContactPoint);

        // Wheel hub position along suspension travel line
        wheel.wheelWorldPosition.copy(this._worldMount).addScaledVector(this._bodyDown, cfg.restLength - wheel.compression);

        wheel.wasGrounded = true;
      } else {
        // Airborne branch
        wheel.grounded = false;
        wheel.wasGrounded = false;
        wheel.compression = 0;
        wheel.compressionVelocity = 0;
        wheel.previousCompression = 0;
        wheel.normalLoad = 0;
        wheel.overTravel = 0;
        wheel.previousOverTravel = 0;
        wheel.overTravelVelocity = 0;
        wheel.dominantRayId = 'airborne';
        wheel.wheelWorldPosition.copy(this._worldMount).addScaledVector(this._bodyDown, cfg.restLength);
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

      // Bottom-out hard stop force
      let hardStopForce = 0;
      if (wheel.overTravel > 0) {
        hardStopForce = this.hardStopStiffness * wheel.overTravel + this.hardStopDamping * wheel.overTravelVelocity;
        hardStopForce = clamp(hardStopForce, 0, this.hardStopMaxForce);
      } else {
        hardStopForce = 0;
      }

      // Anti-roll bar adjustment
      let arbDelta = 0;
      if (wheel.id === 'FL') arbDelta = -frontArbForce;
      else if (wheel.id === 'FR') arbDelta = frontArbForce;
      else if (wheel.id === 'RL') arbDelta = -rearArbForce;
      else if (wheel.id === 'RR') arbDelta = rearArbForce;

      const rawLoad = springForce + dampingForce + bumpStopForce + hardStopForce + arbDelta;
      wheel.normalLoad = Math.max(0, rawLoad);

      // Suspension force vector: push along contact surface normal
      this._forceVec.copy(wheel.contactNormal).multiplyScalar(wheel.normalLoad);
      rigidBody.applyForceAtWorldPoint(this._forceVec, wheel.contactPoint);
    }
  }
}
