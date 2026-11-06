/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import * as THREE from 'three';
import {
  SCRATCH_QUAT_A,
  SCRATCH_MAT3_A,
  SCRATCH_MAT3_B,
  SCRATCH_MAT4_A,
  isFiniteNumber,
} from '../utils/math';

export interface RigidBodyConfig {
  mass: number;             // kg (default: 1800)
  dimensions: THREE.Vector3;// width (x: 1.8), height (y: 1.9), length (z: 4.0)
  comOffset: THREE.Vector3; // offset from geometric center (e.g. 0, -0.22, 0.12)
  linearDamping: number;    // air resistance drag factor (default: 0.0)
  angularDamping: number;   // rotational damping factor (default: 0.15)
  gravity: number;          // m/s^2 (default: 9.81)
}

export interface SafePose {
  position: THREE.Vector3;
  quaternion: THREE.Quaternion;
  linearVelocity: THREE.Vector3;
  angularVelocity: THREE.Vector3;
}

export class RigidBody {
  // Primary state
  public readonly position = new THREE.Vector3();
  public readonly quaternion = new THREE.Quaternion();
  public readonly linearVelocity = new THREE.Vector3();
  public readonly angularVelocity = new THREE.Vector3();

  // Configuration constants
  public readonly mass: number;
  public readonly invMass: number;
  public readonly dimensions = new THREE.Vector3();
  public readonly comOffset = new THREE.Vector3();
  public linearDamping: number;
  public angularDamping: number;
  public gravity: number;

  // Inertia tensors
  public readonly inertiaTensorBody = new THREE.Vector3();    // Principal diagonal [Ixx, Iyy, Izz]
  public readonly invInertiaTensorBody = new THREE.Vector3(); // [1/Ixx, 1/Iyy, 1/Izz]
  private readonly invInertiaWorld = new THREE.Matrix3();     // Rotated into world space

  // Force and Torque accumulators
  private readonly accumulatedForce = new THREE.Vector3();
  private readonly accumulatedTorque = new THREE.Vector3();

  // DEDICATED private scratch vectors to guarantee ZERO caller-variable collision
  private readonly _comLocal = new THREE.Vector3();
  private readonly _comWorld = new THREE.Vector3();
  private readonly _rWorld = new THREE.Vector3();
  private readonly _torqueWorld = new THREE.Vector3();
  private readonly _tangentVel = new THREE.Vector3();
  private readonly _gravityVec = new THREE.Vector3();
  private readonly _linearAccel = new THREE.Vector3();
  private readonly _angularAccel = new THREE.Vector3();

  // Safety & NaN recovery
  private readonly lastSafePose: SafePose;
  public nanResetCount: number = 0;
  public lastResetTime: number = 0;

  constructor(config: Partial<RigidBodyConfig> = {}) {
    this.mass = config.mass ?? 1800.0;
    this.invMass = 1.0 / this.mass;

    this.dimensions.copy(config.dimensions ?? new THREE.Vector3(1.8, 1.9, 4.0));
    // Center of mass offset: slightly lower (-0.22m) and slightly forward (+0.12m along +z)
    this.comOffset.copy(config.comOffset ?? new THREE.Vector3(0.0, -0.22, 0.12));

    this.linearDamping = config.linearDamping ?? 0.0;
    // ITEM 7 CHECK: angular damping set to 0.15 (prevents over-damping yaw)
    this.angularDamping = config.angularDamping ?? 0.15;
    this.gravity = config.gravity ?? 9.81;

    // ITEM 7 CHECK: For 1.8 x 1.9 x 4.0 m box of 1800 kg:
    // Ixx (Pitch) = (1/12) * m * (h^2 + l^2) = 150 * (3.61 + 16.0) = 2941.5 kg*m^2
    // Iyy (Yaw)   = (1/12) * m * (w^2 + l^2) = 150 * (3.24 + 16.0) = 2886.0 kg*m^2 (~2900 kg*m^2)
    // Izz (Roll)  = (1/12) * m * (w^2 + h^2) = 150 * (3.24 + 3.61) = 1027.5 kg*m^2
    const Ixx = (1.0 / 12.0) * this.mass * (this.dimensions.y ** 2 + this.dimensions.z ** 2);
    const Iyy = (1.0 / 12.0) * this.mass * (this.dimensions.x ** 2 + this.dimensions.z ** 2);
    const Izz = (1.0 / 12.0) * this.mass * (this.dimensions.x ** 2 + this.dimensions.y ** 2);

    this.inertiaTensorBody.set(Ixx, Iyy, Izz);
    this.invInertiaTensorBody.set(1.0 / Ixx, 1.0 / Iyy, 1.0 / Izz);

    this.lastSafePose = {
      position: this.position.clone(),
      quaternion: this.quaternion.clone(),
      linearVelocity: this.linearVelocity.clone(),
      angularVelocity: this.angularVelocity.clone(),
    };
  }

  public resetTo(pos: THREE.Vector3, headingRad: number = 0): void {
    this.position.copy(pos);
    this.quaternion.setFromAxisAngle(new THREE.Vector3(0, 1, 0), headingRad);
    this.linearVelocity.set(0, 0, 0);
    this.angularVelocity.set(0, 0, 0);
    this.clearAccumulators();
    this.recordSafePose();
  }

  public clearAccumulators(): void {
    this.accumulatedForce.set(0, 0, 0);
    this.accumulatedTorque.set(0, 0, 0);
  }

  /**
   * ITEM 1 CHECK: Returns world position of Center of Mass (including comOffset).
   * r_com = body_position + R * comOffset
   */
  public getWorldCOM(target: THREE.Vector3): THREE.Vector3 {
    return target.copy(this.comOffset).applyQuaternion(this.quaternion).add(this.position);
  }

  public localToWorld(localPoint: THREE.Vector3, target: THREE.Vector3): THREE.Vector3 {
    return target.copy(localPoint).applyQuaternion(this.quaternion).add(this.position);
  }

  public worldToLocal(worldPoint: THREE.Vector3, target: THREE.Vector3): THREE.Vector3 {
    return target.copy(worldPoint).sub(this.position).applyQuaternion(this.quaternion.clone().invert());
  }

  /**
   * ITEM 3 CHECK: Contact velocity in world space:
   * v_contact = v_body + omega_body x (r_contact - r_com)
   */
  public getPointVelocity(worldPoint: THREE.Vector3, target: THREE.Vector3): THREE.Vector3 {
    this.getWorldCOM(this._comWorld);
    const r = this._rWorld.copy(worldPoint).sub(this._comWorld);
    const tangentialVel = this._tangentVel.crossVectors(this.angularVelocity, r);
    return target.copy(this.linearVelocity).add(tangentialVel);
  }

  public applyForce(force: THREE.Vector3): void {
    this.accumulatedForce.add(force);
  }

  public applyTorque(torque: THREE.Vector3): void {
    this.accumulatedTorque.add(torque);
  }

  /**
   * ITEM 1 FIX & CHECK:
   * Every tire force is applied at that wheel's world-space contact point.
   * Torque = (r_contact - r_com) x F.
   * Uses dedicated private variables so incoming 'force' is NEVER corrupted or overwritten.
   */
  public applyForceAtWorldPoint(force: THREE.Vector3, worldPoint: THREE.Vector3): void {
    this.accumulatedForce.add(force);

    this.getWorldCOM(this._comWorld);
    const r = this._rWorld.copy(worldPoint).sub(this._comWorld);
    const torque = this._torqueWorld.crossVectors(r, force);
    this.accumulatedTorque.add(torque);
  }

  public applyImpulseAtWorldPoint(impulse: THREE.Vector3, worldPoint: THREE.Vector3): void {
    const dV = this._linearAccel.copy(impulse).multiplyScalar(this.invMass);
    this.linearVelocity.add(dV);

    this.getWorldCOM(this._comWorld);
    const r = this._rWorld.copy(worldPoint).sub(this._comWorld);
    const torqueImpulse = this._torqueWorld.crossVectors(r, impulse);
    this.updateWorldInertia();
    torqueImpulse.applyMatrix3(this.invInertiaWorld);
    this.angularVelocity.add(torqueImpulse);
  }

  private updateWorldInertia(): void {
    const rotMat = SCRATCH_MAT3_A.setFromMatrix4(
      SCRATCH_MAT4_A.makeRotationFromQuaternion(this.quaternion)
    );

    const invDiag = SCRATCH_MAT3_B.set(
      this.invInertiaTensorBody.x, 0, 0,
      0, this.invInertiaTensorBody.y, 0,
      0, 0, this.invInertiaTensorBody.z
    );

    this.invInertiaWorld.multiplyMatrices(rotMat, invDiag);
    const rotTranspose = SCRATCH_MAT3_A.transpose();
    this.invInertiaWorld.multiply(rotTranspose);
  }

  public integrate(dt: number): void {
    // 1. Gravity
    const gravityForce = this._gravityVec.set(0, -this.mass * this.gravity, 0);
    this.accumulatedForce.add(gravityForce);

    // 2. Linear integration
    const linearAccel = this._linearAccel.copy(this.accumulatedForce).multiplyScalar(this.invMass);
    this.linearVelocity.addScaledVector(linearAccel, dt);

    const linearDampFactor = Math.max(0, 1.0 - this.linearDamping * dt);
    this.linearVelocity.multiplyScalar(linearDampFactor);
    this.position.addScaledVector(this.linearVelocity, dt);

    // 3. Angular integration
    this.updateWorldInertia();
    const angularAccel = this._angularAccel.copy(this.accumulatedTorque).applyMatrix3(this.invInertiaWorld);
    this.angularVelocity.addScaledVector(angularAccel, dt);

    const angularDampFactor = Math.max(0, 1.0 - this.angularDamping * dt);
    this.angularVelocity.multiplyScalar(angularDampFactor);

    // Quaternion integration
    const w = this.angularVelocity;
    const q = this.quaternion;
    const dq = SCRATCH_QUAT_A.set(
      0.5 * (w.x * q.w + w.y * q.z - w.z * q.y) * dt,
      0.5 * (w.y * q.w + w.z * q.x - w.x * q.z) * dt,
      0.5 * (w.z * q.w + w.x * q.y - w.y * q.x) * dt,
      0.5 * (-w.x * q.x - w.y * q.y - w.z * q.z) * dt
    );

    q.x += dq.x;
    q.y += dq.y;
    q.z += dq.z;
    q.w += dq.w;
    q.normalize();

    if (!this.checkSanity()) {
      this.recoverFromNaN();
    } else {
      if (this.linearVelocity.lengthSq() < 400 && this.angularVelocity.lengthSq() < 100) {
        this.recordSafePose();
      }
    }

    this.clearAccumulators();
  }

  private checkSanity(): boolean {
    const okPos = isFiniteNumber(this.position.x) && isFiniteNumber(this.position.y) && isFiniteNumber(this.position.z);
    const okQuat = isFiniteNumber(this.quaternion.x) && isFiniteNumber(this.quaternion.y) && isFiniteNumber(this.quaternion.z) && isFiniteNumber(this.quaternion.w);
    const okLinVel = isFiniteNumber(this.linearVelocity.x) && isFiniteNumber(this.linearVelocity.y) && isFiniteNumber(this.linearVelocity.z);
    const okAngVel = isFiniteNumber(this.angularVelocity.x) && isFiniteNumber(this.angularVelocity.y) && isFiniteNumber(this.angularVelocity.z);
    const boundedPos = Math.abs(this.position.x) < 20000 && Math.abs(this.position.y) < 2000 && Math.abs(this.position.z) < 20000;
    return okPos && okQuat && okLinVel && okAngVel && boundedPos;
  }

  private recordSafePose(): void {
    this.lastSafePose.position.copy(this.position);
    this.lastSafePose.quaternion.copy(this.quaternion);
    this.lastSafePose.linearVelocity.copy(this.linearVelocity);
    this.lastSafePose.angularVelocity.copy(this.angularVelocity);
  }

  private recoverFromNaN(): void {
    this.nanResetCount++;
    this.lastResetTime = performance.now();
    this.position.copy(this.lastSafePose.position);
    this.quaternion.copy(this.lastSafePose.quaternion);
    this.linearVelocity.set(0, 0, 0);
    this.angularVelocity.set(0, 0, 0);
    this.clearAccumulators();
  }
}
