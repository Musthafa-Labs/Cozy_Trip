/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import * as THREE from 'three';
import { RigidBody } from './rigidBody';
import { SuspensionSystem } from './suspension';
import { WheelDynamics } from './wheel';

export class VehicleVisualModel {
  public readonly group: THREE.Group;
  private readonly bodyMesh: THREE.Group;
  private readonly wheelSteerGroups: THREE.Group[] = [];
  private readonly wheelSpinGroups: THREE.Group[] = [];

  // Preallocated structures to eliminate per-frame allocations
  private readonly _steerQuat = new THREE.Quaternion();
  private readonly _upAxis = new THREE.Vector3(0, 1, 0);

  constructor() {
    this.group = new THREE.Group();
    this.bodyMesh = new THREE.Group();
    this.group.add(this.bodyMesh);

    this.buildBody();
    this.buildWheels();
  }

  private buildBody(): void {
    const bodyMat = new THREE.MeshStandardMaterial({
      color: 0x6e7f5e, // Vintage olive sage
      roughness: 0.7,
      metalness: 0.1,
      flatShading: true,
    });

    const roofMat = new THREE.MeshStandardMaterial({
      color: 0xf0ece1, // Cream white vintage roof
      roughness: 0.8,
      metalness: 0.05,
      flatShading: true,
    });

    const darkTrimMat = new THREE.MeshStandardMaterial({
      color: 0x2b2f2d, // Matte dark steel/bumper
      roughness: 0.8,
      metalness: 0.2,
      flatShading: true,
    });

    const glassMat = new THREE.MeshStandardMaterial({
      color: 0x7fa2a8, // Lo-fi soft teal glass
      roughness: 0.2,
      metalness: 0.4,
      transparent: true,
      opacity: 0.85,
      flatShading: true,
    });

    const headlightMat = new THREE.MeshStandardMaterial({
      color: 0xffe9ad,
      emissive: 0xffdf88,
      emissiveIntensity: 0.6,
      roughness: 0.3,
    });

    const skidMat = new THREE.MeshStandardMaterial({
      color: 0x3a3e3c, // Heavy-duty steel skid plate
      roughness: 0.75,
      metalness: 0.35,
      flatShading: true,
    });

    // 1. Lower chassis / body tub
    const lowerBodyGeom = new THREE.BoxGeometry(1.8, 0.65, 3.8);
    const lowerBody = new THREE.Mesh(lowerBodyGeom, bodyMat);
    lowerBody.position.set(0, 0.2, 0);
    lowerBody.castShadow = true;
    lowerBody.receiveShadow = true;
    this.bodyMesh.add(lowerBody);

    // 1b. Lower body rocker sills (between wheel arches, Z = -0.85 to +0.85m, Y = -0.48m)
    const rockerGeom = new THREE.BoxGeometry(1.76, 0.32, 1.70);
    const rockerMesh = new THREE.Mesh(rockerGeom, bodyMat);
    rockerMesh.position.set(0, -0.32, 0);
    rockerMesh.castShadow = true;
    this.bodyMesh.add(rockerMesh);

    // 1c. Wheel Arch Fender Flares (framing tires at X = ±0.90, Z = ±1.30m)
    const flareGeom = new THREE.BoxGeometry(0.08, 0.22, 0.90);
    for (const zAxle of [1.30, -1.30]) {
      for (const signX of [-1, 1]) {
        const flare = new THREE.Mesh(flareGeom, darkTrimMat);
        flare.position.set(signX * 0.90, -0.28, zAxle);
        flare.castShadow = true;
        this.bodyMesh.add(flare);
      }
    }

    // 1d. Underbody Chassis Frame Rails & Center Belly Pan
    // Center transfer case / transmission skid plate (Y = -0.70m, 0.32m clearance on level ground)
    const bellyPanGeom = new THREE.BoxGeometry(0.90, 0.08, 1.40);
    const bellyPan = new THREE.Mesh(bellyPanGeom, skidMat);
    bellyPan.position.set(0, -0.70, 0);
    bellyPan.castShadow = true;
    bellyPan.receiveShadow = true;
    this.bodyMesh.add(bellyPan);

    // Chassis frame rails (longitudinal ladder frame)
    const frameRailGeom = new THREE.BoxGeometry(1.20, 0.16, 3.30);
    const frameRails = new THREE.Mesh(frameRailGeom, darkTrimMat);
    frameRails.position.set(0, -0.52, 0);
    frameRails.castShadow = true;
    this.bodyMesh.add(frameRails);

    // 1e. Front and Rear Solid Axle Tubes & Differential Pumpkins
    const axleTubeGeom = new THREE.CylinderGeometry(0.045, 0.045, 1.45, 8);
    axleTubeGeom.rotateZ(Math.PI / 2);

    const diffPumpkinGeom = new THREE.SphereGeometry(0.12, 10, 8);

    // Front Axle (Z = +1.30m, hub center Y = -0.68m, diff bottom Y = -0.78m)
    const frontAxle = new THREE.Mesh(axleTubeGeom, darkTrimMat);
    frontAxle.position.set(0, -0.68, 1.30);
    frontAxle.castShadow = true;
    this.bodyMesh.add(frontAxle);

    const frontDiff = new THREE.Mesh(diffPumpkinGeom, skidMat);
    frontDiff.position.set(0, -0.70, 1.30);
    frontDiff.scale.set(1.0, 0.85, 1.1);
    frontDiff.castShadow = true;
    this.bodyMesh.add(frontDiff);

    // Rear Axle (Z = -1.30m, hub center Y = -0.68m, diff bottom Y = -0.78m)
    const rearAxle = new THREE.Mesh(axleTubeGeom, darkTrimMat);
    rearAxle.position.set(0, -0.68, -1.30);
    rearAxle.castShadow = true;
    this.bodyMesh.add(rearAxle);

    const rearDiff = new THREE.Mesh(diffPumpkinGeom, skidMat);
    rearDiff.position.set(0, -0.70, -1.30);
    rearDiff.scale.set(1.0, 0.85, 1.1);
    rearDiff.castShadow = true;
    this.bodyMesh.add(rearDiff);

    // 1f. Front Approach Skid Plate (swept up from front axle to front bumper, ~16° ramp)
    const frontSkidGeom = new THREE.BoxGeometry(0.75, 0.06, 0.72);
    frontSkidGeom.rotateX(0.25); // ~14.3° tilt matching approach ramp
    const frontSkid = new THREE.Mesh(frontSkidGeom, skidMat);
    frontSkid.position.set(0, -0.64, 1.70);
    frontSkid.castShadow = true;
    this.bodyMesh.add(frontSkid);

    // 1g. Rear Departure Skid Plate (swept up from rear axle to rear bumper, ~17° ramp)
    const rearSkidGeom = new THREE.BoxGeometry(0.75, 0.06, 0.70);
    rearSkidGeom.rotateX(-0.25);
    const rearSkid = new THREE.Mesh(rearSkidGeom, skidMat);
    rearSkid.position.set(0, -0.63, -1.68);
    rearSkid.castShadow = true;
    this.bodyMesh.add(rearSkid);

    // 2. Cabin / Greenhouse
    const cabinGeom = new THREE.BoxGeometry(1.6, 0.85, 2.2);
    const cabin = new THREE.Mesh(cabinGeom, roofMat);
    cabin.position.set(0, 0.95, -0.45);
    cabin.castShadow = true;
    cabin.receiveShadow = true;
    this.bodyMesh.add(cabin);

    // 3. Cabin windows
    const windshieldGeom = new THREE.BoxGeometry(1.48, 0.55, 0.1);
    const windshield = new THREE.Mesh(windshieldGeom, glassMat);
    windshield.position.set(0, 0.95, 0.66);
    this.bodyMesh.add(windshield);

    const rearGlassGeom = new THREE.BoxGeometry(1.48, 0.55, 0.1);
    const rearGlass = new THREE.Mesh(rearGlassGeom, glassMat);
    rearGlass.position.set(0, 0.95, -1.56);
    this.bodyMesh.add(rearGlass);

    // 4. Bumpers
    // Front Bumper: upper beam and lower chin apron down to Y = -0.56m (approach angle >= 28°)
    const frontBumperGeom = new THREE.BoxGeometry(1.90, 0.25, 0.35);
    const frontBumper = new THREE.Mesh(frontBumperGeom, darkTrimMat);
    frontBumper.position.set(0, 0.05, 2.05);
    frontBumper.castShadow = true;
    this.bodyMesh.add(frontBumper);

    const frontApronGeom = new THREE.BoxGeometry(1.70, 0.38, 0.28);
    const frontApron = new THREE.Mesh(frontApronGeom, darkTrimMat);
    frontApron.position.set(0, -0.26, 2.02);
    frontApron.castShadow = true;
    this.bodyMesh.add(frontApron);

    // Rear Bumper: upper beam and lower departure apron down to Y = -0.54m (departure angle >= 33°)
    const rearBumperGeom = new THREE.BoxGeometry(1.90, 0.25, 0.30);
    const rearBumper = new THREE.Mesh(rearBumperGeom, darkTrimMat);
    rearBumper.position.set(0, 0.05, -1.95);
    rearBumper.castShadow = true;
    this.bodyMesh.add(rearBumper);

    const rearApronGeom = new THREE.BoxGeometry(1.70, 0.38, 0.25);
    const rearApron = new THREE.Mesh(rearApronGeom, darkTrimMat);
    rearApron.position.set(0, -0.26, -1.92);
    rearApron.castShadow = true;
    this.bodyMesh.add(rearApron);

    // 5. Classic Round Headlights
    const headlightGeom = new THREE.CylinderGeometry(0.18, 0.18, 0.1, 16);
    headlightGeom.rotateX(Math.PI / 2);

    const lightL = new THREE.Mesh(headlightGeom, headlightMat);
    lightL.position.set(-0.55, 0.25, 1.95);
    this.bodyMesh.add(lightL);

    const lightR = new THREE.Mesh(headlightGeom, headlightMat);
    lightR.position.set(0.55, 0.25, 1.95);
    this.bodyMesh.add(lightR);

    // 6. Roof rack
    const rackBarGeom = new THREE.BoxGeometry(1.5, 0.08, 2.1);
    const rackBar = new THREE.Mesh(rackBarGeom, darkTrimMat);
    rackBar.position.set(0, 1.42, -0.45);
    this.bodyMesh.add(rackBar);

    // Spare wheel on rear hatch
    const spareGeom = new THREE.CylinderGeometry(0.36, 0.36, 0.22, 16);
    spareGeom.rotateX(Math.PI / 2);
    const spareWheel = new THREE.Mesh(spareGeom, darkTrimMat);
    spareWheel.position.set(0, 0.45, -2.05);
    this.bodyMesh.add(spareWheel);
  }

  private buildWheels(): void {
    const tireMat = new THREE.MeshStandardMaterial({
      color: 0x222423,
      roughness: 0.9,
      metalness: 0.1,
      flatShading: true,
    });

    const rimMat = new THREE.MeshStandardMaterial({
      color: 0xdfdad0,
      roughness: 0.6,
      metalness: 0.3,
      flatShading: true,
    });

    const spokeMat = new THREE.MeshStandardMaterial({
      color: 0xd97736,
      roughness: 0.4,
      metalness: 0.1,
      flatShading: true,
    });

    const lugMat = new THREE.MeshStandardMaterial({
      color: 0xffffff,
      roughness: 0.3,
      metalness: 0.5,
    });

    for (let i = 0; i < 4; i++) {
      const steerGroup = new THREE.Group();
      const spinGroup = new THREE.Group();
      steerGroup.add(spinGroup);

      const tireGeom = new THREE.CylinderGeometry(0.38, 0.38, 0.28, 16);
      tireGeom.rotateZ(Math.PI / 2);
      const tireMesh = new THREE.Mesh(tireGeom, tireMat);
      tireMesh.castShadow = true;
      spinGroup.add(tireMesh);

      const rimGeom = new THREE.CylinderGeometry(0.24, 0.24, 0.29, 12);
      rimGeom.rotateZ(Math.PI / 2);
      const rimMesh = new THREE.Mesh(rimGeom, rimMat);
      spinGroup.add(rimMesh);

      // PART A.2: change rim face offset from "isLeft ? -0.15 : 0.15" to "isLeft ? 0.15 : -0.15"
      const isLeft = i === 0 || i === 2; // FL (0) and RL (2)
      const faceOffset = isLeft ? 0.15 : -0.15;

      const spokeGeom = new THREE.BoxGeometry(0.04, 0.40, 0.03);
      const spoke1 = new THREE.Mesh(spokeGeom, spokeMat);
      spoke1.position.x = faceOffset;
      spinGroup.add(spoke1);

      const spoke2 = new THREE.Mesh(spokeGeom, spokeMat);
      spoke2.position.x = faceOffset;
      spoke2.rotation.x = Math.PI / 2;
      spinGroup.add(spoke2);

      const hubNutGeom = new THREE.CylinderGeometry(0.06, 0.06, 0.05, 6);
      hubNutGeom.rotateZ(Math.PI / 2);
      const hubNut = new THREE.Mesh(hubNutGeom, lugMat);
      hubNut.position.x = faceOffset * 1.05;
      spinGroup.add(hubNut);

      this.group.add(steerGroup);
      this.wheelSteerGroups.push(steerGroup);
      this.wheelSpinGroups.push(spinGroup);
    }
  }

  /**
   * Updates visual mesh transforms from rigid body, suspension, and wheel rotational dynamics.
   * PART A.2: use +dynWheel.steerAngle and preallocated Quaternion.
   */
  public update(
    rigidBody: RigidBody,
    suspension: SuspensionSystem,
    wheelDynamics: WheelDynamics[],
    customBodyPos?: THREE.Vector3,
    customBodyQuat?: THREE.Quaternion,
    customWheelPositions?: THREE.Vector3[],
    customSteerAngles?: number[],
    customSpinAngles?: number[]
  ): void {
    const bodyPos = customBodyPos ?? rigidBody.position;
    const bodyQuat = customBodyQuat ?? rigidBody.quaternion;

    this.bodyMesh.position.copy(bodyPos);
    this.bodyMesh.quaternion.copy(bodyQuat);

    for (let i = 0; i < 4; i++) {
      const suspWheel = suspension.wheels[i];
      const dynWheel = wheelDynamics[i];
      const steerGroup = this.wheelSteerGroups[i];
      const spinGroup = this.wheelSpinGroups[i];

      const wheelPos = customWheelPositions ? customWheelPositions[i] : suspWheel.wheelWorldPosition;
      steerGroup.position.copy(wheelPos);

      // Match body rotation, then apply local Ackermann steer angle (+dynWheel.steerAngle)
      steerGroup.quaternion.copy(bodyQuat);
      const steerAngle = customSteerAngles ? customSteerAngles[i] : dynWheel.steerAngle;
      if (steerAngle !== 0) {
        this._steerQuat.setFromAxisAngle(this._upAxis, steerAngle);
        steerGroup.quaternion.multiply(this._steerQuat);
      }

      const spinAngle = customSpinAngles ? customSpinAngles[i] : dynWheel.spinAngle;
      spinGroup.rotation.x = spinAngle;
    }
  }
}
