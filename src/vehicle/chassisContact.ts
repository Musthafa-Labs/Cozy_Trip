/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import * as THREE from 'three';
import { RigidBody } from './rigidBody';
import { TestGround, ObstacleContactResult } from '../world/testGround';
import { SurfaceType } from './tire';
import { SuspensionSystem } from './suspension';
import { WheelDynamics } from './wheel';
import { SCRATCH_MAT3_A, SCRATCH_MAT3_B, SCRATCH_MAT4_A } from '../utils/math';

export interface ChassisContactConfig {
  stiffness: number;       // Penalty spring stiffness (N/m)
  damping: number;         // Normal damping (N*s/m)
  mu: number;              // Tangential friction coefficient for body/roof (0.6)
  muUnderbody: number;     // Sliding friction coefficient for underbody skid plates (0.28)
  restitution: number;     // Restitution coefficient on obstacle impact (0.12)
}

export class ChassisContactSystem {
  // Comprehensive vehicle chassis collision hull points in local body space (+Z forward, +X left, +Y up)
  // Aligned with the visible vehicle underbody, skid plates, wheel arches, and bumpers:
  // - Front bumper & approach apron: swept from Z = 1.45m up to Z = 2.225m, Y = -0.72m to -0.05m (>=28° approach)
  // - Front & rear diff pumpkins: Y = -0.78m (exact 0.28m clearance on level ground at Y_body = 1.06m)
  // - Center belly pan / transfer case: Y = -0.74m (0.32m clearance on level ground)
  // - Rocker panels / sills: Y = -0.66m (0.40m clearance on level ground)
  // - Rear departure apron: swept from Z = -1.45m to -2.10m, Y = -0.72m to -0.05m (>=33° departure)
  // - Wheel arch clearance: arches up to Y = -0.28m over wheels (0.40m above hub, full travel clearance)
  // - Roof & roof rack: Y = 1.375m and Y = 1.46m (stable rollover support)
  public readonly localPoints: THREE.Vector3[] = [];
  public readonly isUnderbodyPoint: boolean[] = [];

  public stiffness: number;
  public damping: number;
  public mu: number;
  public muUnderbody: number;
  public restitution: number;

  // Real-time telemetry for HUD
  public penetratingContactCount: number = 0;
  public maxPenetrationMeters: number = 0;
  public safetyNetCounter: number = 0;

  // Dedicated private scratch vectors to guarantee ZERO allocations in physics loop
  private readonly _worldPoint = new THREE.Vector3();
  private readonly _groundNormal = new THREE.Vector3();
  private readonly _pointVel = new THREE.Vector3();
  private readonly _vTangent = new THREE.Vector3();
  private readonly _frictionForce = new THREE.Vector3();
  private readonly _totalForce = new THREE.Vector3();
  private readonly _worldCOM = new THREE.Vector3();
  private readonly _rWorld = new THREE.Vector3();
  private readonly _rxn = new THREE.Vector3();
  private readonly _invInertiaRxn = new THREE.Vector3();
  private readonly _impulseVec = new THREE.Vector3();
  private readonly _invInertiaWorld = new THREE.Matrix3();
  private readonly _dominantObsNormal = new THREE.Vector3();
  private readonly _bodyUp = new THREE.Vector3(0, 1, 0);
  private readonly _wheelQuat = new THREE.Quaternion();
  private readonly _wheelSteerQuat = new THREE.Quaternion();
  private readonly _wheelAxle = new THREE.Vector3();
  private readonly _wheelLong = new THREE.Vector3();
  private readonly _wheelUp = new THREE.Vector3();

  // Preallocated buffer for active obstacle contacts in a single step (up to 350 points)
  private readonly _collidingWorldPoints: THREE.Vector3[] = [];
  private readonly _collidingNormals: THREE.Vector3[] = [];
  private readonly _collidingPens: number[] = [];
  private readonly _collidingIsUnderbody: boolean[] = [];

  // Preallocated buffer for active terrain ground contacts in a single step (up to 350 points)
  private readonly _terrainWorldPoints: THREE.Vector3[] = [];
  private readonly _terrainNormals: THREE.Vector3[] = [];
  private readonly _terrainPens: number[] = [];
  private readonly _terrainIsUnderbody: boolean[] = [];

  private readonly _obsResult: ObstacleContactResult = {
    penetration: 0,
    normal: new THREE.Vector3(),
    surface: 'asphalt' as SurfaceType,
    pointOnObstacle: new THREE.Vector3(),
  };

  constructor(config: Partial<ChassisContactConfig> = {}) {
    this.stiffness = config.stiffness ?? 220000;
    this.damping = config.damping ?? 8000;
    this.mu = config.mu ?? 0.60;
    this.muUnderbody = config.muUnderbody ?? 0.28;
    this.restitution = config.restitution ?? 0.12;

    this.buildLocalHullPoints();

    const maxBufferPts = 350;
    for (let i = 0; i < maxBufferPts; i++) {
      this._collidingWorldPoints.push(new THREE.Vector3());
      this._collidingNormals.push(new THREE.Vector3());
      this._collidingPens.push(0);
      this._collidingIsUnderbody.push(false);

      this._terrainWorldPoints.push(new THREE.Vector3());
      this._terrainNormals.push(new THREE.Vector3());
      this._terrainPens.push(0);
      this._terrainIsUnderbody.push(false);
    }
  }

  /**
   * Generates hull contact points that match the physical boundary of the visible vehicle.
   */
  private buildLocalHullPoints(): void {
    const addPt = (x: number, y: number, z: number, isUnderbody: boolean = false) => {
      this.localPoints.push(new THREE.Vector3(x, y, z));
      this.isUnderbodyPoint.push(isUnderbody);
    };

    // =========================================================================
    // 1. UNDERBODY SKID PLATES & AXLE DIFFERENTIALS (isUnderbody = true)
    // On stationary level road (body Y = 1.06m):
    // - Diffs at Y = -0.78m -> world height Y = 0.28m (specified 0.28m diff clearance)
    // - Center belly pan at Y = -0.74m -> world height Y = 0.32m (breakover clearance)
    // - Axle tubes at Y = -0.70m -> world height Y = 0.36m
    // =========================================================================

    // Front Differential Pumpkin & Axle (Z = +1.30m)
    addPt( 0.00, -0.78, 1.30, true);
    addPt(-0.15, -0.77, 1.30, true);
    addPt( 0.15, -0.77, 1.30, true);
    addPt( 0.00, -0.76, 1.22, true);
    addPt( 0.00, -0.76, 1.38, true);
    addPt(-0.45, -0.70, 1.30, true);
    addPt( 0.45, -0.70, 1.30, true);

    // Front Swept Approach Skid Plate (Z = 1.45 to 2.05m, ramp angle ~17°)
    // Angled from front axle (Y = -0.74m) up to front bumper lower lip (Y = -0.52m)
    addPt( 0.00, -0.72, 1.45, true);
    addPt(-0.35, -0.72, 1.45, true);
    addPt( 0.35, -0.72, 1.45, true);

    addPt( 0.00, -0.65, 1.65, true);
    addPt(-0.45, -0.65, 1.65, true);
    addPt( 0.45, -0.65, 1.65, true);

    addPt( 0.00, -0.58, 1.85, true);
    addPt(-0.55, -0.58, 1.85, true);
    addPt( 0.55, -0.58, 1.85, true);

    addPt( 0.00, -0.52, 2.05, true);
    addPt(-0.65, -0.52, 2.05, true);
    addPt( 0.65, -0.52, 2.05, true);

    // Center Belly Pan / Transfer Case Skid Plate (Z = -0.60m to +0.60m, Y = -0.74m)
    const zBelly = [-0.60, -0.30, 0.00, 0.30, 0.60];
    for (const z of zBelly) {
      addPt(-0.40, -0.74, z, true);
      addPt( 0.00, -0.74, z, true);
      addPt( 0.40, -0.74, z, true);
    }
    // Subframe transitions from belly pan to axles (Z = ±0.95m, Y = -0.73m)
    addPt(-0.30, -0.73,  0.95, true);
    addPt( 0.00, -0.73,  0.95, true);
    addPt( 0.30, -0.73,  0.95, true);
    addPt(-0.30, -0.73, -0.95, true);
    addPt( 0.00, -0.73, -0.95, true);
    addPt( 0.30, -0.73, -0.95, true);

    // Rear Differential Pumpkin & Axle (Z = -1.30m)
    addPt( 0.00, -0.78, -1.30, true);
    addPt(-0.15, -0.77, -1.30, true);
    addPt( 0.15, -0.77, -1.30, true);
    addPt( 0.00, -0.76, -1.22, true);
    addPt( 0.00, -0.76, -1.38, true);
    addPt(-0.45, -0.70, -1.30, true);
    addPt( 0.45, -0.70, -1.30, true);

    // Rear Departure Skid Plate (Z = -1.45 to -1.95m, ramp angle ~18°)
    // Angled from rear axle (Y = -0.74m) up to rear bumper lower lip (Y = -0.52m)
    addPt( 0.00, -0.71, -1.45, true);
    addPt(-0.35, -0.71, -1.45, true);
    addPt( 0.35, -0.71, -1.45, true);

    addPt( 0.00, -0.65, -1.65, true);
    addPt(-0.45, -0.65, -1.65, true);
    addPt( 0.45, -0.65, -1.65, true);

    addPt( 0.00, -0.58, -1.85, true);
    addPt(-0.55, -0.58, -1.85, true);
    addPt( 0.55, -0.58, -1.85, true);

    addPt( 0.00, -0.52, -1.98, true);
    addPt(-0.65, -0.52, -1.98, true);
    addPt( 0.65, -0.52, -1.98, true);

    // Rocker Panels / Side Sills (X = ±0.88m, Z = -0.75 to +0.75m, Y = -0.66m, world height 0.40m)
    const zRockers = [-0.75, -0.38, 0.00, 0.38, 0.75];
    for (const z of zRockers) {
      addPt( 0.88, -0.66, z, true);
      addPt(-0.88, -0.66, z, true);
    }

    // =========================================================================
    // 2. FRONT BUMPER: Face at Z = 2.225m, Width X = ±0.95m
    // Swept approach angle >= 34° clears 20° parking hill easily
    // =========================================================================
    const xFrontBumper = [-0.95, -0.65, -0.35, 0.00, 0.35, 0.65, 0.95];
    for (const y of [-0.42, -0.26, -0.10, 0.05, 0.175]) {
      for (const x of xFrontBumper) {
        addPt(x, y, 2.225, false);
      }
    }
    // Front bumper intermediate apron (Z = 2.05m)
    for (const y of [-0.52, -0.34, -0.14]) {
      for (const x of [-0.85, -0.45, 0.00, 0.45, 0.85]) {
        addPt(x, y, 2.05, false);
      }
    }
    // Front bumper side wraps (X = ±0.95m, Z from 1.95 to 2.225m)
    for (const z of [1.95, 2.10, 2.225]) {
      for (const y of [-0.44, -0.20, 0.05]) {
        addPt( 0.95, y, z, false);
        addPt(-0.95, y, z, false);
      }
    }
    // Headlights and hood front
    addPt(-0.55, 0.25, 2.00, false);
    addPt( 0.55, 0.25, 2.00, false);
    for (const x of [-0.75, 0.00, 0.75]) {
      addPt(x, 0.525, 1.85, false);
    }

    // =========================================================================
    // 3. REAR BUMPER & SPARE TIRE: Rear face at Z = -2.10m, Spare at Z = -2.16m
    // Swept departure angle >= 34°
    // =========================================================================
    const xRearBumper = [-0.95, -0.65, -0.35, 0.00, 0.35, 0.65, 0.95];
    for (const y of [-0.44, -0.28, -0.10, 0.05, 0.175]) {
      for (const x of xRearBumper) {
        addPt(x, y, -2.10, false);
      }
    }
    // Rear bumper intermediate apron (Z = -1.95m)
    for (const y of [-0.52, -0.34, -0.14]) {
      for (const x of [-0.85, -0.45, 0.00, 0.45, 0.85]) {
        addPt(x, y, -1.95, false);
      }
    }
    for (const z of [-1.85, -1.98, -2.10]) {
      for (const y of [-0.44, -0.20, 0.05]) {
        addPt( 0.95, y, z, false);
        addPt(-0.95, y, z, false);
      }
    }
    // Spare tire on rear hatch (Z = -2.16m)
    addPt( 0.00, 0.45, -2.16, false);
    addPt( 0.25, 0.45, -2.16, false);
    addPt(-0.25, 0.45, -2.16, false);
    addPt( 0.00, 0.20, -2.15, false);
    addPt( 0.00, 0.70, -2.15, false);

    // =========================================================================
    // 4. WHEEL ARCHES & UPPER BODY SIDES: Full clearance above 0.38m tires
    // Wheel arch crest at Y = -0.28m (0.40m above axle hub at -0.68m)
    // =========================================================================
    for (const zAxle of [1.30, -1.30]) {
      addPt( 0.90, -0.28, zAxle, false);
      addPt(-0.90, -0.28, zAxle, false);
      addPt( 0.90, -0.34, zAxle - 0.35, false);
      addPt(-0.90, -0.34, zAxle - 0.35, false);
      addPt( 0.90, -0.34, zAxle + 0.35, false);
      addPt(-0.90, -0.34, zAxle + 0.35, false);
    }

    // Body waist (Y = 0.20m and 0.50m) along sides (X = ±0.90m)
    const zBodySides = [-1.80, -1.20, -0.60, 0.00, 0.60, 1.20, 1.80];
    for (const z of zBodySides) {
      addPt( 0.90, 0.20, z, false);
      addPt(-0.90, 0.20, z, false);
      addPt( 0.90, 0.50, z, false);
      addPt(-0.90, 0.50, z, false);
    }

    // =========================================================================
    // 5. CABIN GREENHOUSE, ROOF & ROOF RACK (Rollover stability)
    // =========================================================================
    // Windshield base & top
    addPt( 0.00, 0.95,  0.66, false);
    addPt( 0.74, 0.95,  0.66, false);
    addPt(-0.74, 0.95,  0.66, false);
    addPt( 0.00, 0.95, -1.56, false);
    addPt( 0.74, 0.95, -1.56, false);
    addPt(-0.74, 0.95, -1.56, false);

    // Roof perimeter (Y = 1.375m)
    for (const z of [-1.55, -0.80, 0.00, 0.65]) {
      addPt( 0.00, 1.375, z, false);
      addPt( 0.80, 1.375, z, false);
      addPt(-0.80, 1.375, z, false);
    }

    // Roof rack top (Y = 1.46m)
    addPt( 0.75, 1.46,  0.60, false);
    addPt(-0.75, 1.46,  0.60, false);
    addPt( 0.75, 1.46, -0.45, false);
    addPt(-0.75, 1.46, -0.45, false);
    addPt( 0.75, 1.46, -1.50, false);
    addPt(-0.75, 1.46, -1.50, false);
  }

  public reset(): void {
    this.penetratingContactCount = 0;
    this.maxPenetrationMeters = 0;
  }

  /**
   * Helper to compute world-space inverse inertia tensor.
   */
  private updateWorldInertia(rigidBody: RigidBody): void {
    const rotMat = SCRATCH_MAT3_A.setFromMatrix4(
      SCRATCH_MAT4_A.makeRotationFromQuaternion(rigidBody.quaternion)
    );

    const invDiag = SCRATCH_MAT3_B.set(
      rigidBody.invInertiaTensorBody.x, 0, 0,
      0, rigidBody.invInertiaTensorBody.y, 0,
      0, 0, rigidBody.invInertiaTensorBody.z
    );

    this._invInertiaWorld.multiplyMatrices(rotMat, invDiag);
    const rotTranspose = SCRATCH_MAT3_A.transpose();
    this._invInertiaWorld.multiply(rotTranspose);
  }

  /**
   * Evaluates obstacle contacts for all 4 dynamic wheels using live world transforms.
   */
  private collectWheelContacts(
    rigidBody: RigidBody,
    ground: TestGround,
    suspension: SuspensionSystem,
    wheelsDyn?: WheelDynamics[],
    onContact?: (worldPt: THREE.Vector3, pen: number, norm: THREE.Vector3) => void
  ): { count: number; maxPen: number; domNormal: THREE.Vector3 } {
    let wheelContactCount = 0;
    let maxWheelPen = 0;
    const domNorm = new THREE.Vector3();

    const wheelRadius = 0.38;
    const halfWidth = 0.14;
    const rimOffset = 0.16;

    // 12 angles around cylinder perimeter (30 deg increments)
    const angles12 = [
      0, Math.PI / 6, Math.PI / 3, Math.PI / 2, (2 * Math.PI) / 3, (5 * Math.PI) / 6,
      Math.PI, (7 * Math.PI) / 6, (4 * Math.PI) / 3, (3 * Math.PI) / 2, (5 * Math.PI) / 3, (11 * Math.PI) / 6
    ];

    this._bodyUp.set(0, 1, 0).applyQuaternion(rigidBody.quaternion);

    for (let i = 0; i < 4; i++) {
      const suspWheel = suspension.wheels[i];
      const steerAngle = (i < 2 && wheelsDyn) ? wheelsDyn[i].steerAngle : 0;
      const isLeft = i === 0 || i === 2; // FL (0) or RL (2)
      const outerSign = isLeft ? 1 : -1;

      this._wheelQuat.copy(rigidBody.quaternion);
      if (steerAngle !== 0) {
        this._wheelSteerQuat.setFromAxisAngle(this._bodyUp, steerAngle);
        this._wheelQuat.multiply(this._wheelSteerQuat);
      }

      this._wheelAxle.set(outerSign, 0, 0).applyQuaternion(this._wheelQuat);
      this._wheelLong.set(0, 0, 1).applyQuaternion(this._wheelQuat);
      this._wheelUp.set(0, 1, 0).applyQuaternion(this._wheelQuat);

      const wheelPos = suspWheel.wheelWorldPosition;

      // 1. Outer rim/sidewall perimeter
      for (const theta of angles12) {
        const sinT = Math.sin(theta);
        const cosT = Math.cos(theta);
        this._worldPoint.copy(wheelPos)
          .addScaledVector(this._wheelAxle, rimOffset)
          .addScaledVector(this._wheelLong, sinT * wheelRadius)
          .addScaledVector(this._wheelUp, cosT * wheelRadius);

        if (ground.getObstacleContact(this._worldPoint, this._obsResult)) {
          const pen = this._obsResult.penetration;
          if (pen > 0) {
            wheelContactCount++;
            if (pen > maxWheelPen) {
              maxWheelPen = pen;
              domNorm.copy(this._obsResult.normal);
            }
            if (onContact) onContact(this._worldPoint, pen, this._obsResult.normal);
          }
        }
      }

      // 2. Tread center line
      for (const theta of angles12) {
        const sinT = Math.sin(theta);
        const cosT = Math.cos(theta);
        this._worldPoint.copy(wheelPos)
          .addScaledVector(this._wheelLong, sinT * wheelRadius)
          .addScaledVector(this._wheelUp, cosT * wheelRadius);

        if (ground.getObstacleContact(this._worldPoint, this._obsResult)) {
          const pen = this._obsResult.penetration;
          if (pen > 0) {
            wheelContactCount++;
            if (pen > maxWheelPen) {
              maxWheelPen = pen;
              domNorm.copy(this._obsResult.normal);
            }
            if (onContact) onContact(this._worldPoint, pen, this._obsResult.normal);
          }
        }
      }

      // 3. Inner sidewall perimeter
      for (let a = 0; a < 6; a++) {
        const theta = a * (Math.PI / 3);
        const sinT = Math.sin(theta);
        const cosT = Math.cos(theta);
        this._worldPoint.copy(wheelPos)
          .addScaledVector(this._wheelAxle, -halfWidth)
          .addScaledVector(this._wheelLong, sinT * wheelRadius)
          .addScaledVector(this._wheelUp, cosT * wheelRadius);

        if (ground.getObstacleContact(this._worldPoint, this._obsResult)) {
          const pen = this._obsResult.penetration;
          if (pen > 0) {
            wheelContactCount++;
            if (pen > maxWheelPen) {
              maxWheelPen = pen;
              domNorm.copy(this._obsResult.normal);
            }
            if (onContact) onContact(this._worldPoint, pen, this._obsResult.normal);
          }
        }
      }

      // 4. Outer rim center
      this._worldPoint.copy(wheelPos).addScaledVector(this._wheelAxle, rimOffset);
      if (ground.getObstacleContact(this._worldPoint, this._obsResult)) {
        const pen = this._obsResult.penetration;
        if (pen > 0) {
          wheelContactCount++;
          if (pen > maxWheelPen) {
            maxWheelPen = pen;
            domNorm.copy(this._obsResult.normal);
          }
          if (onContact) onContact(this._worldPoint, pen, this._obsResult.normal);
        }
      }
    }

    return { count: wheelContactCount, maxPen: maxWheelPen, domNormal: domNorm };
  }

  /**
   * Pre-integration step:
   * Evaluates terrain penetration, penalty forces, tangential friction,
   * obstacle rigid-body impulses, and sustained reaction forces.
   */
  public update(
    rigidBody: RigidBody,
    ground: TestGround,
    dt: number,
    suspension?: SuspensionSystem,
    wheelsDyn?: WheelDynamics[]
  ): void {
    const fixedDt = dt > 0 ? dt : 1 / 120;
    let penetratingCount = 0;
    let deepestGroundPenetration = 0;
    let deepestObsPenetration = 0;
    let terrainContactCount = 0;
    let obsContactCount = 0;
    this._dominantObsNormal.set(0, 0, 0);

    const numPoints = this.localPoints.length;
    rigidBody.getWorldCOM(this._worldCOM);
    this.updateWorldInertia(rigidBody);

    // -------------------------------------------------------------------------
    // A. Evaluate Chassis Hull Points (Terrain & Obstacles)
    // -------------------------------------------------------------------------
    for (let i = 0; i < numPoints; i++) {
      rigidBody.localToWorld(this.localPoints[i], this._worldPoint);
      const isUnderbody = this.isUnderbodyPoint[i];

      // 1. Terrain Surface Height Query
      const groundInfo = ground.getHeightNormalSurface(this._worldPoint.x, this._worldPoint.z);
      const groundHeight = groundInfo.height;
      const groundPen = groundHeight - this._worldPoint.y;

      if (groundPen > 0) {
        penetratingCount++;
        if (groundPen > deepestGroundPenetration) {
          deepestGroundPenetration = groundPen;
        }

        if (terrainContactCount < this._terrainWorldPoints.length) {
          this._terrainWorldPoints[terrainContactCount].copy(this._worldPoint);
          this._terrainNormals[terrainContactCount].copy(groundInfo.normal);
          this._terrainPens[terrainContactCount] = groundPen;
          this._terrainIsUnderbody[terrainContactCount] = isUnderbody;
          terrainContactCount++;
        }
      }

      // 2. 3D Solid Static Obstacle Contact (Cube)
      if (ground.getObstacleContact(this._worldPoint, this._obsResult)) {
        const obsPen = this._obsResult.penetration;
        if (obsPen > 0) {
          penetratingCount++;
          if (obsPen > deepestObsPenetration) {
            deepestObsPenetration = obsPen;
            this._dominantObsNormal.copy(this._obsResult.normal);
          }

          if (obsContactCount < this._collidingWorldPoints.length) {
            this._collidingWorldPoints[obsContactCount].copy(this._worldPoint);
            this._collidingNormals[obsContactCount].copy(this._obsResult.normal);
            this._collidingPens[obsContactCount] = obsPen;
            this._collidingIsUnderbody[obsContactCount] = isUnderbody;
            obsContactCount++;
          }
        }
      }
    }

    // -------------------------------------------------------------------------
    // Safety-net correction for severe TERRAIN GROUND penetration (falling through)
    // Only triggers on severe drops (>0.55m into terrain)
    // -------------------------------------------------------------------------
    if (deepestGroundPenetration > 0.55) {
      const lift = deepestGroundPenetration - 0.05;
      rigidBody.position.y += lift;
      rigidBody.clearAccumulators();
      if (rigidBody.linearVelocity.y < 0) {
        rigidBody.linearVelocity.y = 0;
      }
      rigidBody.linearVelocity.x *= 0.5;
      rigidBody.linearVelocity.z *= 0.5;
      rigidBody.angularVelocity.multiplyScalar(0.2);
      this.safetyNetCounter++;
      this.penetratingContactCount = penetratingCount;
      this.maxPenetrationMeters = Math.max(deepestGroundPenetration, deepestObsPenetration);
      return;
    }

    // -------------------------------------------------------------------------
    // B. Resolve Terrain Ground Contacts (Load-Shared, Dissipative, Smooth Sliding)
    // -------------------------------------------------------------------------
    if (terrainContactCount > 0) {
      const countScale = 1.0 / Math.max(1, Math.sqrt(terrainContactCount));
      const kPoint = this.stiffness * countScale;
      const cPoint = this.damping * countScale;

      for (let j = 0; j < terrainContactCount; j++) {
        const pt = this._terrainWorldPoints[j];
        const normal = this._terrainNormals[j];
        const pen = this._terrainPens[j];
        const isUnderbody = this._terrainIsUnderbody[j];
        const pointMu = isUnderbody ? this.muUnderbody : this.mu;

        rigidBody.getPointVelocity(pt, this._pointVel);
        const vNormal = this._pointVel.dot(normal);

        const r = this._rWorld.copy(pt).sub(this._worldCOM);
        const rxn = this._rxn.crossVectors(r, normal);
        const invInertiaRxn = this._invInertiaRxn.copy(rxn).applyMatrix3(this._invInertiaWorld);
        const angularTerm = this._vTangent.crossVectors(invInertiaRxn, r).dot(normal);
        const mEffInv = rigidBody.invMass + Math.max(0, angularTerm);
        const mEff = 1.0 / mEffInv;

        // Damping: strictly dissipative, preventing any velocity reversal
        let dampForce = 0;
        if (vNormal < 0) {
          const maxDamp = 0.85 * (mEff * Math.abs(vNormal) / fixedDt);
          dampForce = Math.min(cPoint * Math.abs(vNormal), maxDamp);
        } else {
          dampForce = -Math.min(cPoint * vNormal, kPoint * pen);
        }

        const maxNormalPerPoint = (rigidBody.mass * 25.0) / Math.max(1, terrainContactCount * 0.5);
        const normalForce = Math.max(0, Math.min(maxNormalPerPoint, kPoint * pen + dampForce));

        this._totalForce.copy(normal).multiplyScalar(normalForce);

        this._vTangent.copy(this._pointVel).addScaledVector(normal, -vNormal);
        const tanSpeed = this._vTangent.length();

        if (tanSpeed > 1e-4) {
          const tanDir = this._vTangent.normalize();
          const rxt = this._rxn.crossVectors(r, tanDir);
          const invInertiaRxt = this._invInertiaRxn.copy(rxt).applyMatrix3(this._invInertiaWorld);
          const tanAngularTerm = this._rxn.crossVectors(invInertiaRxt, r).dot(tanDir);
          const mEffTanInv = rigidBody.invMass + Math.max(0, tanAngularTerm);
          const mEffTan = 1.0 / mEffTanInv;

          // Underbody skid plates use muUnderbody (0.28) for smooth sliding over crests/logs
          const maxFrictionCoulomb = pointMu * normalForce;
          const maxFrictionStop = 0.85 * (mEffTan * tanSpeed / fixedDt);
          const frictionMag = Math.min(maxFrictionCoulomb, maxFrictionStop);

          this._totalForce.addScaledVector(tanDir, -frictionMag);
        }

        rigidBody.applyForceAtWorldPoint(this._totalForce, pt);
      }
    }

    // -------------------------------------------------------------------------
    // C. Evaluate Wheel Cylinders against Obstacles
    // -------------------------------------------------------------------------
    if (suspension) {
      this.collectWheelContacts(rigidBody, ground, suspension, wheelsDyn, (worldPt, pen, norm) => {
        penetratingCount++;
        if (pen > deepestObsPenetration) {
          deepestObsPenetration = pen;
          this._dominantObsNormal.copy(norm);
        }
        if (obsContactCount < this._collidingWorldPoints.length) {
          this._collidingWorldPoints[obsContactCount].copy(worldPt);
          this._collidingNormals[obsContactCount].copy(norm);
          this._collidingPens[obsContactCount] = pen;
          this._collidingIsUnderbody[obsContactCount] = false;
          obsContactCount++;
        }
      });
    }

    // -------------------------------------------------------------------------
    // D. Resolve Obstacle Contact Impulses & Reaction Forces
    // -------------------------------------------------------------------------
    if (obsContactCount > 0) {
      for (let j = 0; j < obsContactCount; j++) {
        const pt = this._collidingWorldPoints[j];
        const normal = this._collidingNormals[j];
        const pen = this._collidingPens[j];
        const isUnderbody = this._collidingIsUnderbody[j];
        const pointMu = isUnderbody ? this.muUnderbody : this.mu;

        rigidBody.getPointVelocity(pt, this._pointVel);
        const vNormal = this._pointVel.dot(normal);

        const r = this._rWorld.copy(pt).sub(this._worldCOM);
        const rxn = this._rxn.crossVectors(r, normal);
        const invInertiaRxn = this._invInertiaRxn.copy(rxn).applyMatrix3(this._invInertiaWorld);
        const angularTerm = this._vTangent.crossVectors(invInertiaRxn, r).dot(normal);
        const mEffInv = rigidBody.invMass + Math.max(0, angularTerm);

        if (vNormal < 0) {
          const e = Math.abs(vNormal) > 0.4 ? this.restitution : 0.0;
          const targetDeltaV = -(1.0 + e) * vNormal;
          const jNormal = Math.max(0, targetDeltaV / (mEffInv * Math.max(1, obsContactCount * 0.4)));

          this._vTangent.copy(this._pointVel).addScaledVector(normal, -vNormal);
          const tangentSpeed = this._vTangent.length();
          this._impulseVec.copy(normal).multiplyScalar(jNormal);

          if (tangentSpeed > 1e-4) {
            const tangentDir = this._vTangent.normalize();
            const rxt = this._rxn.crossVectors(r, tangentDir);
            const invInertiaRxt = this._invInertiaRxn.copy(rxt).applyMatrix3(this._invInertiaWorld);
            const tanAngularTerm = this._pointVel.crossVectors(invInertiaRxt, r).dot(tangentDir);
            const mEffTanInv = rigidBody.invMass + Math.max(0, tanAngularTerm);

            const jTangentMax = pointMu * jNormal;
            const jTangent = Math.min(jTangentMax, tangentSpeed / mEffTanInv);
            this._impulseVec.addScaledVector(tangentDir, -jTangent);
          }

          rigidBody.applyImpulseAtWorldPoint(this._impulseVec, pt);
        }

        const normalForce = Math.max(0, this.stiffness * 2.0 * pen);
        rigidBody.getPointVelocity(pt, this._pointVel);
        const currentVn = this._pointVel.dot(normal);
        this._vTangent.copy(this._pointVel).addScaledVector(normal, -currentVn);
        const tanSpeed = this._vTangent.length();
        const denom = Math.max(tanSpeed, 0.2);
        this._frictionForce.copy(this._vTangent).multiplyScalar((-pointMu * normalForce) / denom);

        this._totalForce.copy(normal).multiplyScalar(normalForce).add(this._frictionForce);
        rigidBody.applyForceAtWorldPoint(this._totalForce, pt);
      }
    }

    this.penetratingContactCount = penetratingCount;
    this.maxPenetrationMeters = Math.max(deepestGroundPenetration, deepestObsPenetration);
  }

  /**
   * Post-integration step:
   * Called IMMEDIATELY after rigidBody.integrate() and before recording render interpolation state.
   * Enforces zero residual penetration against solid obstacles and aligns suspension wheels.
   */
  public postIntegrate(
    rigidBody: RigidBody,
    ground: TestGround,
    suspension?: SuspensionSystem,
    wheelsDyn?: WheelDynamics[]
  ): void {
    let maxPen = 0;
    const domNormal = this._dominantObsNormal;
    domNormal.set(0, 0, 0);

    const numPoints = this.localPoints.length;

    // 1. Check chassis points post-integration
    for (let i = 0; i < numPoints; i++) {
      rigidBody.localToWorld(this.localPoints[i], this._worldPoint);
      if (ground.getObstacleContact(this._worldPoint, this._obsResult)) {
        const pen = this._obsResult.penetration;
        if (pen > maxPen) {
          maxPen = pen;
          domNormal.copy(this._obsResult.normal);
        }
      }
    }

    // 2. Check wheel points post-integration
    if (suspension) {
      const wheelRes = this.collectWheelContacts(rigidBody, ground, suspension, wheelsDyn);
      if (wheelRes.maxPen > maxPen) {
        maxPen = wheelRes.maxPen;
        domNormal.copy(wheelRes.domNormal);
      }
    }

    // 3. Exact non-penetration position projection against solid static obstacle
    if (maxPen > 0 && domNormal.lengthSq() > 0.5) {
      rigidBody.position.addScaledVector(domNormal, maxPen);

      const vNorm = rigidBody.linearVelocity.dot(domNormal);
      if (vNorm < 0) {
        rigidBody.linearVelocity.addScaledVector(domNormal, -vNorm);
      }

      if (suspension) {
        const bodyDown = this._pointVel.set(0, -1, 0).applyQuaternion(rigidBody.quaternion).normalize();
        for (let i = 0; i < 4; i++) {
          const wheel = suspension.wheels[i];
          rigidBody.localToWorld(wheel.config.localMountPoint, this._worldPoint);
          wheel.wheelWorldPosition.copy(this._worldPoint).addScaledVector(bodyDown, wheel.config.restLength - wheel.compression);
        }
      }

      this.maxPenetrationMeters = 0;
    } else {
      this.maxPenetrationMeters = maxPen;
    }
  }
}

