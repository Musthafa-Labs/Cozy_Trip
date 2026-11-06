/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import * as THREE from 'three';
import { RigidBody } from '../vehicle/rigidBody';
import { SuspensionSystem } from '../vehicle/suspension';
import { WheelDynamics } from '../vehicle/wheel';
import { SCRATCH_VEC3_A, SCRATCH_VEC3_B } from '../utils/math';

export class DebugVisualizer {
  public readonly group: THREE.Group;
  public visible: boolean = true;

  private readonly lines: THREE.LineSegments;
  private readonly positions: Float32Array;
  private readonly colors: Float32Array;

  private readonly comMarker: THREE.Mesh;

  constructor() {
    this.group = new THREE.Group();

    // 4 suspension rays (8 vertices)
    // 4 contact normals (8 vertices)
    // 4 longitudinal force arrows (8 vertices)
    // 4 lateral force arrows (8 vertices)
    // Total 32 vertices * 3 = 96 floats
    const maxVertices = 32;
    this.positions = new Float32Array(maxVertices * 3);
    this.colors = new Float32Array(maxVertices * 3);

    const lineGeom = new THREE.BufferGeometry();
    lineGeom.setAttribute('position', new THREE.BufferAttribute(this.positions, 3));
    lineGeom.setAttribute('color', new THREE.BufferAttribute(this.colors, 3));

    const lineMat = new THREE.LineBasicMaterial({
      vertexColors: true,
      linewidth: 2,
      depthTest: false,
      transparent: true,
      opacity: 0.9,
    });

    this.lines = new THREE.LineSegments(lineGeom, lineMat);
    this.lines.renderOrder = 999;
    this.group.add(this.lines);

    // Center of Mass sphere marker
    const comGeom = new THREE.SphereGeometry(0.1, 8, 8);
    const comMat = new THREE.MeshBasicMaterial({ color: 0xff3333, wireframe: true, depthTest: false });
    this.comMarker = new THREE.Mesh(comGeom, comMat);
    this.comMarker.renderOrder = 1000;
    this.group.add(this.comMarker);
  }

  public update(
    rigidBody: RigidBody,
    suspension: SuspensionSystem,
    wheelDynamics: WheelDynamics[]
  ): void {
    if (!this.visible) {
      this.group.visible = false;
      return;
    }
    this.group.visible = true;

    // 1. Center of mass position
    const com = SCRATCH_VEC3_A;
    rigidBody.getWorldCOM(com);
    this.comMarker.position.copy(com);

    let vIdx = 0;

    for (let i = 0; i < 4; i++) {
      const suspWheel = suspension.wheels[i];
      const dynWheel = wheelDynamics[i];
      const worldMount = SCRATCH_VEC3_B;
      rigidBody.localToWorld(suspWheel.config.localMountPoint, worldMount);

      // -------------------------------------------------------------
      // Line 1: Mount to Wheel Position / Contact (Green/Blue)
      // -------------------------------------------------------------
      this.positions[vIdx * 3] = worldMount.x;
      this.positions[vIdx * 3 + 1] = worldMount.y;
      this.positions[vIdx * 3 + 2] = worldMount.z;

      const r = suspWheel.grounded ? 0.2 : 0.2;
      const g = suspWheel.grounded ? 0.9 : 0.4;
      const b = suspWheel.grounded ? 0.3 : 1.0;

      this.colors[vIdx * 3] = r;
      this.colors[vIdx * 3 + 1] = g;
      this.colors[vIdx * 3 + 2] = b;
      vIdx++;

      const endPt = suspWheel.grounded ? suspWheel.contactPoint : suspWheel.wheelWorldPosition;
      this.positions[vIdx * 3] = endPt.x;
      this.positions[vIdx * 3 + 1] = endPt.y;
      this.positions[vIdx * 3 + 2] = endPt.z;

      this.colors[vIdx * 3] = r;
      this.colors[vIdx * 3 + 1] = g;
      this.colors[vIdx * 3 + 2] = b;
      vIdx++;

      if (suspWheel.grounded) {
        // -------------------------------------------------------------
        // Line 2: Contact Normal (Yellow/Orange)
        // -------------------------------------------------------------
        this.positions[vIdx * 3] = suspWheel.contactPoint.x;
        this.positions[vIdx * 3 + 1] = suspWheel.contactPoint.y;
        this.positions[vIdx * 3 + 2] = suspWheel.contactPoint.z;

        this.colors[vIdx * 3] = 1.0;
        this.colors[vIdx * 3 + 1] = 0.8;
        this.colors[vIdx * 3 + 2] = 0.1;
        vIdx++;

        this.positions[vIdx * 3] = suspWheel.contactPoint.x + suspWheel.contactNormal.x * 0.45;
        this.positions[vIdx * 3 + 1] = suspWheel.contactPoint.y + suspWheel.contactNormal.y * 0.45;
        this.positions[vIdx * 3 + 2] = suspWheel.contactPoint.z + suspWheel.contactNormal.z * 0.45;

        this.colors[vIdx * 3] = 1.0;
        this.colors[vIdx * 3 + 1] = 0.4;
        this.colors[vIdx * 3 + 2] = 0.1;
        vIdx++;

        // -------------------------------------------------------------
        // Line 3: Longitudinal Tire Force Fx (RED)
        // Scale: 1000 N = 0.35m arrow
        // -------------------------------------------------------------
        const fxScale = (dynWheel.Fx / 2500) * 0.6;
        this.positions[vIdx * 3] = suspWheel.contactPoint.x;
        this.positions[vIdx * 3 + 1] = suspWheel.contactPoint.y + 0.04;
        this.positions[vIdx * 3 + 2] = suspWheel.contactPoint.z;

        this.colors[vIdx * 3] = 0.95;
        this.colors[vIdx * 3 + 1] = 0.2;
        this.colors[vIdx * 3 + 2] = 0.2;
        vIdx++;

        this.positions[vIdx * 3] = suspWheel.contactPoint.x + dynWheel.worldLongitudinalDir.x * fxScale;
        this.positions[vIdx * 3 + 1] = suspWheel.contactPoint.y + 0.04 + dynWheel.worldLongitudinalDir.y * fxScale;
        this.positions[vIdx * 3 + 2] = suspWheel.contactPoint.z + dynWheel.worldLongitudinalDir.z * fxScale;

        this.colors[vIdx * 3] = 1.0;
        this.colors[vIdx * 3 + 1] = 0.3;
        this.colors[vIdx * 3 + 2] = 0.3;
        vIdx++;

        // -------------------------------------------------------------
        // Line 4: Lateral Tire Force Fy (BLUE/CYAN)
        // Scale: 1000 N = 0.35m arrow
        // -------------------------------------------------------------
        const fyScale = (dynWheel.Fy / 2500) * 0.6;
        this.positions[vIdx * 3] = suspWheel.contactPoint.x;
        this.positions[vIdx * 3 + 1] = suspWheel.contactPoint.y + 0.04;
        this.positions[vIdx * 3 + 2] = suspWheel.contactPoint.z;

        this.colors[vIdx * 3] = 0.2;
        this.colors[vIdx * 3 + 1] = 0.5;
        this.colors[vIdx * 3 + 2] = 1.0;
        vIdx++;

        this.positions[vIdx * 3] = suspWheel.contactPoint.x + dynWheel.worldLateralDir.x * fyScale;
        this.positions[vIdx * 3 + 1] = suspWheel.contactPoint.y + 0.04 + dynWheel.worldLateralDir.y * fyScale;
        this.positions[vIdx * 3 + 2] = suspWheel.contactPoint.z + dynWheel.worldLateralDir.z * fyScale;

        this.colors[vIdx * 3] = 0.3;
        this.colors[vIdx * 3 + 1] = 0.7;
        this.colors[vIdx * 3 + 2] = 1.0;
        vIdx++;
      } else {
        // Zero unused lines
        for (let k = 0; k < 18; k++) {
          this.positions[vIdx * 3 + k] = 0;
          this.colors[vIdx * 3 + k] = 0;
        }
        vIdx += 6;
      }
    }

    const posAttr = this.lines.geometry.attributes.position as THREE.BufferAttribute;
    const colAttr = this.lines.geometry.attributes.color as THREE.BufferAttribute;
    posAttr.needsUpdate = true;
    colAttr.needsUpdate = true;
  }
}
