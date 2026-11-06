/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import * as THREE from 'three';
import { RigidBody } from './rigidBody';
import { TestGround } from '../world/testGround';

export interface ChassisContactConfig {
  stiffness: number; // Penalty spring stiffness (N/m)
  damping: number;   // Normal damping (N*s/m)
  mu: number;        // Tangential friction coefficient (0.6)
}

export class ChassisContactSystem {
  // 12 vehicle chassis collision contact points in local body space (+Z forward, +X left, +Y up)
  // Vehicle body dimensions: 1.8m width, 1.9m height, 4.0m length
  public readonly localPoints: THREE.Vector3[] = [
    // Bottom perimeter (floorpan / bumpers / rockers)
    new THREE.Vector3(0.85, -0.15, 1.95),   // 0: Front-Left Bumper
    new THREE.Vector3(-0.85, -0.15, 1.95),  // 1: Front-Right Bumper
    new THREE.Vector3(0.85, -0.15, -1.95),  // 2: Rear-Left Bumper
    new THREE.Vector3(-0.85, -0.15, -1.95), // 3: Rear-Right Bumper
    new THREE.Vector3(0.90, -0.15, 0.0),    // 4: Mid-Left Rocker
    new THREE.Vector3(-0.90, -0.15, 0.0),   // 5: Mid-Right Rocker

    // Mid/Hood waist perimeter
    new THREE.Vector3(0.75, 0.55, 1.85),   // 6: Hood Front-Left
    new THREE.Vector3(-0.75, 0.55, 1.85),  // 7: Hood Front-Right

    // Roof perimeter (rollover / inverted support)
    new THREE.Vector3(0.75, 1.35, 0.60),   // 8: Roof Front-Left
    new THREE.Vector3(-0.75, 1.35, 0.60),  // 9: Roof Front-Right
    new THREE.Vector3(0.75, 1.35, -1.55),  // 10: Roof Rear-Left
    new THREE.Vector3(-0.75, 1.35, -1.55), // 11: Roof Rear-Right
  ];

  public stiffness: number;
  public damping: number;
  public mu: number;

  // Real-time telemetry for F3 HUD
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

  constructor(config: Partial<ChassisContactConfig> = {}) {
    this.stiffness = config.stiffness ?? 100000;
    this.damping = config.damping ?? 5000;
    this.mu = config.mu ?? 0.6;
  }

  public reset(): void {
    this.penetratingContactCount = 0;
    this.maxPenetrationMeters = 0;
  }

  /**
   * Evaluates chassis contact penetration, penalty spring-damper normal forces,
   * tangential regularized friction, and emergency safety-net correction.
   *
   * Called exactly once per fixed physics step:
   * After suspension.update() and before rigidBody.integrate().
   */
  public update(rigidBody: RigidBody, ground: TestGround, _dt: number): void {
    let penetratingCount = 0;
    let deepestPenetration = 0;

    const numPoints = this.localPoints.length;

    for (let i = 0; i < numPoints; i++) {
      // 1. Transform local contact point to world space
      rigidBody.localToWorld(this.localPoints[i], this._worldPoint);

      // 2. Rule C: Ground query handling
      // Process each returned normal immediately or copy it into a preallocated scratch vector
      // before making another ground query. Do not retain references to returned normals between iterations.
      const groundInfo = ground.getHeightNormalSurface(this._worldPoint.x, this._worldPoint.z);
      this._groundNormal.copy(groundInfo.normal);
      const groundHeight = groundInfo.height;

      // 3. Penetration along vertical / ground plane
      const penetration = groundHeight - this._worldPoint.y;

      if (penetration > 0) {
        penetratingCount++;
        if (penetration > deepestPenetration) {
          deepestPenetration = penetration;
        }

        // 4. Obtain point velocity
        rigidBody.getPointVelocity(this._worldPoint, this._pointVel);

        // 5. Rule A: Calculate normal component
        const vNormal = this._pointVel.dot(this._groundNormal);

        // Normal penalty contact force (repulsive spring + damping)
        const normalForce = Math.max(0, this.stiffness * penetration - this.damping * vNormal);

        // 6. Rule A: Calculate tangential velocity
        // vTangent = pointVelocity - groundNormal * vNormal
        this._vTangent.copy(this._pointVel).addScaledVector(this._groundNormal, -vNormal);

        // 7. Rule A: Regularized tangential friction (mu = 0.6)
        // frictionForce = -mu * normalForce * vTangent / max(vTangent.length(), 0.2)
        const tangentSpeed = this._vTangent.length();
        const denom = Math.max(tangentSpeed, 0.2);
        this._frictionForce.copy(this._vTangent).multiplyScalar((-this.mu * normalForce) / denom);

        // 8. Combine normal and tangential friction forces
        // Friction force remains strictly tangent to ground surface
        this._totalForce.copy(this._groundNormal).multiplyScalar(normalForce).add(this._frictionForce);

        // Apply force to rigid body at world contact point (r_contact - r_com x F applied automatically)
        rigidBody.applyForceAtWorldPoint(this._totalForce, this._worldPoint);
      }
    }

    // 9. Rule B: Safety-net correction
    // Track the deepest penetration across all 12 contact points.
    // If maximum penetration exceeds 0.5 m:
    if (deepestPenetration > 0.5) {
      // Raise the rigid body vertically by enough to bring deepest point to ~0.05 m maximum penetration
      const lift = deepestPenetration - 0.05;
      rigidBody.position.y += lift;

      // If linearVelocity.y is negative, set only linearVelocity.y to zero
      if (rigidBody.linearVelocity.y < 0) {
        rigidBody.linearVelocity.y = 0;
      }
      // Preserve horizontal velocity and angular velocity

      // Increment cumulative safetyNetCounter once for this correction event, not once per contact point
      this.safetyNetCounter++;
    }

    this.penetratingContactCount = penetratingCount;
    this.maxPenetrationMeters = deepestPenetration;
  }
}
