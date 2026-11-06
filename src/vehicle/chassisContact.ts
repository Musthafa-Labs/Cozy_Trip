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
  stiffness: number; // Penalty spring stiffness (N/m)
  damping: number;   // Normal damping (N*s/m)
  mu: number;        // Tangential friction coefficient (0.6)
  restitution: number; // Restitution coefficient on obstacle impact (0.12)
}

export class ChassisContactSystem {
  // Comprehensive vehicle chassis collision hull points in local body space (+Z forward, +X left, +Y up)
  // Perfectly aligned with the visual vehicle model in vehicleModel.ts:
  // - Front bumper: width 1.90m (X: -0.95 to +0.95), height (Y: -0.075 to +0.175), front face at Z = +2.225m
  // - Lower body tub: width 1.80m (X: -0.90 to +0.90), height (Y: -0.125 to +0.525), length (Z: -1.90 to +1.90)
  // - Rear bumper: width 1.90m (X: -0.95 to +0.95), rear face at Z = -2.10m
  // - Spare wheel: rear face at Z = -2.16m
  // - Roof: width 1.60m, height Y = 1.375m, roof rack Y = 1.46m
  public readonly localPoints: THREE.Vector3[] = [];

  public stiffness: number;
  public damping: number;
  public mu: number;
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
  private readonly _wheelPtLocal = new THREE.Vector3();

  // Preallocated buffer for active obstacle contacts in a single step (up to 300 points)
  private readonly _collidingWorldPoints: THREE.Vector3[] = [];
  private readonly _collidingNormals: THREE.Vector3[] = [];
  private readonly _collidingPens: number[] = [];

  private readonly _obsResult: ObstacleContactResult = {
    penetration: 0,
    normal: new THREE.Vector3(),
    surface: 'asphalt' as SurfaceType,
    pointOnObstacle: new THREE.Vector3(),
  };

  constructor(config: Partial<ChassisContactConfig> = {}) {
    this.stiffness = config.stiffness ?? 220000;
    this.damping = config.damping ?? 8000;
    this.mu = config.mu ?? 0.6;
    this.restitution = config.restitution ?? 0.12;

    this.buildLocalHullPoints();

    const maxBufferPts = 350;
    for (let i = 0; i < maxBufferPts; i++) {
      this._collidingWorldPoints.push(new THREE.Vector3());
      this._collidingNormals.push(new THREE.Vector3());
      this._collidingPens.push(0);
    }
  }

  /**
   * Generates hull contact points that exactly match the physical boundary of the visible vehicle.
   */
  private buildLocalHullPoints(): void {
    const pts: THREE.Vector3[] = [];

    // =========================================================================
    // 1. FRONT BUMPER: Box(1.90, 0.25, 0.35) at (0, 0.05, 2.05)
    // Front face is at Z = 2.225m, X: -0.95 to +0.95, Y: -0.075 to +0.175
    // =========================================================================
    const xBumperFront = [-0.95, -0.80, -0.65, -0.50, -0.35, -0.20, 0.00, 0.20, 0.35, 0.50, 0.65, 0.80, 0.95];
    const yBumperFront = [-0.075, 0.05, 0.175];
    for (const y of yBumperFront) {
      for (const x of xBumperFront) {
        pts.push(new THREE.Vector3(x, y, 2.225));
      }
    }

    // Front bumper lateral side faces (X = ±0.95m, Z from 1.875 to 2.225)
    const zBumperSides = [1.875, 1.99, 2.11, 2.225];
    for (const z of zBumperSides) {
      for (const y of [-0.075, 0.05, 0.175]) {
        pts.push(new THREE.Vector3( 0.95, y, z));
        pts.push(new THREE.Vector3(-0.95, y, z));
      }
    }

    // Front chin skid / subframe (Z = 1.95, Y = -0.15)
    for (const x of [-0.85, -0.60, -0.30, 0.00, 0.30, 0.60, 0.85]) {
      pts.push(new THREE.Vector3(x, -0.15, 1.95));
    }

    // Front hood waist & headlights (Z = 1.90 to 2.00, Y = 0.25 to 0.525)
    for (const x of [-0.80, -0.55, -0.25, 0.00, 0.25, 0.55, 0.80]) {
      pts.push(new THREE.Vector3(x, 0.45, 1.90));
      pts.push(new THREE.Vector3(x, 0.525, 1.80));
    }
    pts.push(new THREE.Vector3(-0.55, 0.25, 2.00)); // Left Headlight
    pts.push(new THREE.Vector3( 0.55, 0.25, 2.00)); // Right Headlight

    // =========================================================================
    // 2. VEHICLE SIDES: Lower Body Tub (X = ±0.90m, Z: -1.90 to +1.875)
    // Continuous coverage along rockers (Y = -0.125m) and waist (Y = 0.30m)
    // =========================================================================
    const zSides = [
      -1.90, -1.65, -1.40, -1.15, -0.90, -0.65, -0.40, -0.15,
       0.00,  0.25,  0.50,  0.75,  1.00,  1.25,  1.50,  1.70, 1.875
    ];
    for (const z of zSides) {
      // Left side (+X)
      pts.push(new THREE.Vector3(0.90, -0.125, z));
      pts.push(new THREE.Vector3(0.90,  0.300, z));
      // Right side (-X)
      pts.push(new THREE.Vector3(-0.90, -0.125, z));
      pts.push(new THREE.Vector3(-0.90,  0.300, z));
    }

    // =========================================================================
    // 3. REAR BUMPER & SPARE WHEEL: Rear face at Z = -2.10m, Spare at Z = -2.16m
    // =========================================================================
    const xRear = [-0.95, -0.65, -0.35, 0.00, 0.35, 0.65, 0.95];
    for (const y of [-0.075, 0.05, 0.175]) {
      for (const x of xRear) {
        pts.push(new THREE.Vector3(x, y, -2.10));
      }
    }
    // Rear bumper sides (X = ±0.95m, Z from -1.80 to -2.10)
    for (const z of [-1.80, -1.95, -2.10]) {
      pts.push(new THREE.Vector3( 0.95, 0.05, z));
      pts.push(new THREE.Vector3(-0.95, 0.05, z));
    }
    // Rear spare tire (Z = -2.16m)
    pts.push(new THREE.Vector3( 0.00, 0.45, -2.16));
    pts.push(new THREE.Vector3( 0.25, 0.45, -2.16));
    pts.push(new THREE.Vector3(-0.25, 0.45, -2.16));
    pts.push(new THREE.Vector3( 0.00, 0.20, -2.15));
    pts.push(new THREE.Vector3( 0.00, 0.70, -2.15));

    // Rear lower skid (Z = -1.95, Y = -0.15)
    for (const x of [-0.85, -0.45, 0.00, 0.45, 0.85]) {
      pts.push(new THREE.Vector3(x, -0.15, -1.95));
    }

    // =========================================================================
    // 4. UNDERBODY FLOORPAN: Y = -0.125m
    // =========================================================================
    for (const z of [-1.50, -1.00, -0.50, 0.00, 0.50, 1.00, 1.50]) {
      pts.push(new THREE.Vector3(0.00, -0.125, z));
    }

    // =========================================================================
    // 5. CABIN GREENHOUSE, ROOF & ROOF RACK
    // =========================================================================
    // Windshield base & top
    pts.push(new THREE.Vector3( 0.00, 0.95,  0.66));
    pts.push(new THREE.Vector3( 0.74, 0.95,  0.66));
    pts.push(new THREE.Vector3(-0.74, 0.95,  0.66));
    pts.push(new THREE.Vector3( 0.00, 0.95, -1.56));
    pts.push(new THREE.Vector3( 0.74, 0.95, -1.56));
    pts.push(new THREE.Vector3(-0.74, 0.95, -1.56));

    // Roof perimeter (Y = 1.375m)
    for (const z of [-1.55, -0.80, 0.00, 0.65]) {
      pts.push(new THREE.Vector3( 0.00, 1.375, z));
      pts.push(new THREE.Vector3( 0.80, 1.375, z));
      pts.push(new THREE.Vector3(-0.80, 1.375, z));
    }

    // Roof rack top (Y = 1.46m)
    pts.push(new THREE.Vector3( 0.75, 1.46,  0.60));
    pts.push(new THREE.Vector3(-0.75, 1.46,  0.60));
    pts.push(new THREE.Vector3( 0.75, 1.46, -0.45));
    pts.push(new THREE.Vector3(-0.75, 1.46, -0.45));
    pts.push(new THREE.Vector3( 0.75, 1.46, -1.50));
    pts.push(new THREE.Vector3(-0.75, 1.46, -1.50));

    for (const p of pts) {
      this.localPoints.push(p);
    }
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
    const rimOffset = 0.16; // Outer face of rim/hub nut

    // 12 angles around cylinder perimeter (30 deg increments)
    const angles12 = [0, Math.PI / 6, Math.PI / 3, Math.PI / 2, (2 * Math.PI) / 3, (5 * Math.PI) / 6,
                      Math.PI, (7 * Math.PI) / 6, (4 * Math.PI) / 3, (3 * Math.PI) / 2, (5 * Math.PI) / 3, (11 * Math.PI) / 6];

    this._bodyUp.set(0, 1, 0).applyQuaternion(rigidBody.quaternion);

    for (let i = 0; i < 4; i++) {
      const suspWheel = suspension.wheels[i];
      const steerAngle = (i < 2 && wheelsDyn) ? wheelsDyn[i].steerAngle : 0;
      const isLeft = i === 0 || i === 2; // FL (0) or RL (2)
      const outerSign = isLeft ? 1 : -1;

      // Wheel world orientation: body quaternion * steer angle around local up
      this._wheelQuat.copy(rigidBody.quaternion);
      if (steerAngle !== 0) {
        this._wheelSteerQuat.setFromAxisAngle(this._bodyUp, steerAngle);
        this._wheelQuat.multiply(this._wheelSteerQuat);
      }

      // Wheel local axes in world space:
      // In local wheel space, spin axis is along X, up is along Y, forward rolling is along Z
      this._wheelAxle.set(outerSign, 0, 0).applyQuaternion(this._wheelQuat); // Outer lateral direction
      this._wheelLong.set(0, 0, 1).applyQuaternion(this._wheelQuat);        // Forward rolling direction
      this._wheelUp.set(0, 1, 0).applyQuaternion(this._wheelQuat);          // Upward direction

      const wheelPos = suspWheel.wheelWorldPosition;

      // 1. Outer rim/sidewall perimeter (outermost visual surface of wheel)
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

      // 2. Tread center line (crown of tire)
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
   *
   * Called after suspension.update() and before rigidBody.integrate().
   */
  public update(
    rigidBody: RigidBody,
    ground: TestGround,
    _dt: number,
    suspension?: SuspensionSystem,
    wheelsDyn?: WheelDynamics[]
  ): void {
    let penetratingCount = 0;
    let deepestGroundPenetration = 0;
    let deepestObsPenetration = 0;
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

      // 1. Terrain Surface Height Query & Contact
      const groundInfo = ground.getHeightNormalSurface(this._worldPoint.x, this._worldPoint.z);
      this._groundNormal.copy(groundInfo.normal);
      const groundHeight = groundInfo.height;

      const groundPen = groundHeight - this._worldPoint.y;

      if (groundPen > 0) {
        penetratingCount++;
        if (groundPen > deepestGroundPenetration) {
          deepestGroundPenetration = groundPen;
        }

        rigidBody.getPointVelocity(this._worldPoint, this._pointVel);
        const vNormal = this._pointVel.dot(this._groundNormal);
        const normalForce = Math.max(0, this.stiffness * groundPen - this.damping * vNormal);

        this._vTangent.copy(this._pointVel).addScaledVector(this._groundNormal, -vNormal);
        const tangentSpeed = this._vTangent.length();
        const denom = Math.max(tangentSpeed, 0.2);
        this._frictionForce.copy(this._vTangent).multiplyScalar((-this.mu * normalForce) / denom);

        this._totalForce.copy(this._groundNormal).multiplyScalar(normalForce).add(this._frictionForce);
        rigidBody.applyForceAtWorldPoint(this._totalForce, this._worldPoint);
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
            obsContactCount++;
          }
        }
      }
    }

    // -------------------------------------------------------------------------
    // B. Evaluate Wheel Cylinders against Obstacles
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
          obsContactCount++;
        }
      });
    }

    // -------------------------------------------------------------------------
    // C. Resolve Obstacle Contact Impulses & Reaction Forces
    // -------------------------------------------------------------------------
    if (obsContactCount > 0) {
      for (let j = 0; j < obsContactCount; j++) {
        const pt = this._collidingWorldPoints[j];
        const normal = this._collidingNormals[j];
        const pen = this._collidingPens[j];

        rigidBody.getPointVelocity(pt, this._pointVel);
        const vNormal = this._pointVel.dot(normal);

        // r = pt - worldCOM
        const r = this._rWorld.copy(pt).sub(this._worldCOM);

        // Effective inverse mass along contact normal
        const rxn = this._rxn.crossVectors(r, normal);
        const invInertiaRxn = this._invInertiaRxn.copy(rxn).applyMatrix3(this._invInertiaWorld);
        const angularTerm = this._vTangent.crossVectors(invInertiaRxn, r).dot(normal);
        const mEffInv = rigidBody.invMass + Math.max(0, angularTerm);

        // Dynamic collision impulse: cancels incoming normal velocity
        if (vNormal < 0) {
          const e = Math.abs(vNormal) > 0.4 ? this.restitution : 0.0;
          const targetDeltaV = -(1.0 + e) * vNormal;
          const jNormal = Math.max(0, targetDeltaV / (mEffInv * Math.max(1, obsContactCount * 0.4)));

          // Tangential friction impulse
          this._vTangent.copy(this._pointVel).addScaledVector(normal, -vNormal);
          const tangentSpeed = this._vTangent.length();
          this._impulseVec.copy(normal).multiplyScalar(jNormal);

          if (tangentSpeed > 1e-4) {
            const tangentDir = this._vTangent.normalize();
            const rxt = this._rxn.crossVectors(r, tangentDir);
            const invInertiaRxt = this._invInertiaRxn.copy(rxt).applyMatrix3(this._invInertiaWorld);
            const tanAngularTerm = this._pointVel.crossVectors(invInertiaRxt, r).dot(tangentDir);
            const mEffTanInv = rigidBody.invMass + Math.max(0, tanAngularTerm);

            const jTangentMax = this.mu * jNormal;
            const jTangent = Math.min(jTangentMax, tangentSpeed / mEffTanInv);
            this._impulseVec.addScaledVector(tangentDir, -jTangent);
          }

          rigidBody.applyImpulseAtWorldPoint(this._impulseVec, pt);
        }

        // Sustained normal restoring force (resisting engine drive torque when pressed against cube)
        const normalForce = Math.max(0, this.stiffness * 2.0 * pen);
        rigidBody.getPointVelocity(pt, this._pointVel);
        const currentVn = this._pointVel.dot(normal);
        this._vTangent.copy(this._pointVel).addScaledVector(normal, -currentVn);
        const tanSpeed = this._vTangent.length();
        const denom = Math.max(tanSpeed, 0.2);
        this._frictionForce.copy(this._vTangent).multiplyScalar((-this.mu * normalForce) / denom);

        this._totalForce.copy(normal).multiplyScalar(normalForce).add(this._frictionForce);
        rigidBody.applyForceAtWorldPoint(this._totalForce, pt);
      }
    }

    // -------------------------------------------------------------------------
    // Rule B: Safety-net correction for TERRAIN GROUND penetration only
    // -------------------------------------------------------------------------
    if (deepestGroundPenetration > 0.5) {
      const lift = deepestGroundPenetration - 0.05;
      rigidBody.position.y += lift;

      if (rigidBody.linearVelocity.y < 0) {
        rigidBody.linearVelocity.y = 0;
      }
      this.safetyNetCounter++;
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

    // 3. Exact non-penetration position projection:
    // If any point is inside the obstacle, project rigid body outward so max penetration becomes 0
    if (maxPen > 0 && domNormal.lengthSq() > 0.5) {
      rigidBody.position.addScaledVector(domNormal, maxPen);

      // If linear velocity has inward velocity towards the obstacle, remove it
      const vNorm = rigidBody.linearVelocity.dot(domNormal);
      if (vNorm < 0) {
        rigidBody.linearVelocity.addScaledVector(domNormal, -vNorm);
      }

      // Re-align suspension wheel positions with the projected rigid body
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
