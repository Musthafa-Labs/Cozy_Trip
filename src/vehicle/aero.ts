/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import * as THREE from 'three';
import { RigidBody } from './rigidBody';

export interface AeroConfig {
  airDensity: number;        // kg/m^3 (default: 1.225 at sea level, 15°C)
  dragCoefficient: number;   // Cd (default: 0.52 for vintage upright box 4x4)
  frontalArea: number;       // m^2 (default: 2.85 m^2 based on 1.8m width x 1.9m height with ground clearance)
  downforceCoeff: number;    // Cl downforce coefficient (default: 0.12)
  copOffset: THREE.Vector3;  // Aerodynamic Center of Pressure (COP) offset relative to body COM
}

export const DEFAULT_AERO_CONFIG: AeroConfig = {
  airDensity: 1.225,
  dragCoefficient: 0.52,
  frontalArea: 2.85,
  downforceCoeff: 0.12,
  copOffset: new THREE.Vector3(0, 0.12, -0.15), // Slightly above and behind COM
};

export class Aerodynamics {
  public readonly config: AeroConfig;

  // Live forces for telemetry & inspection (N)
  public dragForceMagnitude: number = 0;
  public downforceMagnitude: number = 0;

  // DEDICATED private instance vectors to guarantee ZERO allocations per frame
  private readonly _worldVel = new THREE.Vector3();
  private readonly _dragForce = new THREE.Vector3();
  private readonly _downforce = new THREE.Vector3();
  private readonly _totalAeroForce = new THREE.Vector3();
  private readonly _worldCOP = new THREE.Vector3();
  private readonly _worldCOM = new THREE.Vector3();

  constructor(config: Partial<AeroConfig> = {}) {
    this.config = {
      ...DEFAULT_AERO_CONFIG,
      ...config,
      copOffset: config.copOffset?.clone() ?? DEFAULT_AERO_CONFIG.copOffset.clone(),
    };
  }

  public reset(): void {
    this.dragForceMagnitude = 0;
    this.downforceMagnitude = 0;
  }

  /**
   * Evaluates aerodynamic drag and downforce and applies them to the rigid body at the Center of Pressure.
   *
   * @param rigidBody The vehicle's 3D rigid body
   */
  public update(rigidBody: RigidBody): void {
    this._worldVel.copy(rigidBody.linearVelocity);
    const speed = this._worldVel.length();

    // At low speeds (< 0.1 m/s), aerodynamic forces are negligible
    if (speed < 0.1) {
      this.dragForceMagnitude = 0;
      this.downforceMagnitude = 0;
      return;
    }

    // 1. Aerodynamic Drag Force: Fdrag = 0.5 * rho * Cd * A * v^2
    // Vector form opposing velocity: F_drag = -0.5 * rho * Cd * A * |v| * v
    const dragCoeff = 0.5 * this.config.airDensity * this.config.dragCoefficient * this.config.frontalArea;
    this._dragForce.copy(this._worldVel).multiplyScalar(-dragCoeff * speed);
    this.dragForceMagnitude = this._dragForce.length();

    // 2. Aerodynamic Downforce: Fdown = 0.5 * rho * Cl * A * v^2
    // Acts strictly downward in world space (-Y) regardless of forward or reverse motion
    const downforceCoeff = 0.5 * this.config.airDensity * this.config.downforceCoeff * this.config.frontalArea;
    const downforceVal = downforceCoeff * speed * speed;
    this._downforce.set(0, -downforceVal, 0);
    this.downforceMagnitude = downforceVal;

    // 3. Combined Aerodynamic Force
    this._totalAeroForce.copy(this._dragForce).add(this._downforce);

    // 4. Center of Pressure (COP) in World Coordinates
    rigidBody.getWorldCOM(this._worldCOM);
    this._worldCOP.copy(this.config.copOffset).applyQuaternion(rigidBody.quaternion).add(this._worldCOM);

    // 5. Apply Force at World Center of Pressure
    rigidBody.applyForceAtWorldPoint(this._totalAeroForce, this._worldCOP);
  }
}
