/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/examples/jsm/geometries/RoundedBoxGeometry.js';
import { RigidBody } from './rigidBody';
import { SuspensionSystem } from './suspension';
import { WheelDynamics } from './wheel';

export interface VehiclePalette {
  bodyColor: number;
  roofColor: number;
  trimColor: number;
  rackColor: number;
  glassColor: number;
  bumperColor: number;
}

/**
 * Palette configuration for the classic 1970s boxy 4x4 wagon.
 * Defaults to vintage olive sage body, cream roof, and matte dark steel trim.
 */
export const VEHICLE_PALETTE: VehiclePalette = {
  bodyColor: 0x6e7f5e,   // Vintage olive sage
  roofColor: 0xf0ece1,   // Cream white vintage roof
  trimColor: 0x2b2f2d,   // Matte dark steel / trim
  rackColor: 0x262928,   // Low-profile roof rack dark steel
  glassColor: 0x7fa2a8,  // Lo-fi soft teal tinted glass
  bumperColor: 0x242726, // Heavy-duty bumper steel
};

/**
 * Static mud splatter layer toggle.
 * When enabled, applies a procedural dark-brown mud splatter layer to the lower 35% of the body panels.
 */
export const ENABLE_MUD_SPLATTER = true;

/**
 * Generates a procedural CanvasTexture with subtle dark-brown mud splatter
 * concentrated on the lower 35% of the texture space, fading upward.
 */
function createMudSplatterTexture(): THREE.CanvasTexture {
  const canvas = document.createElement('canvas');
  canvas.width = 256;
  canvas.height = 256;
  const ctx = canvas.getContext('2d');
  if (!ctx) return new THREE.CanvasTexture(canvas);

  ctx.clearRect(0, 0, 256, 256);

  // Smooth mud gradient along lower 35% (y from 166 to 256)
  const grad = ctx.createLinearGradient(0, 166, 0, 256);
  grad.addColorStop(0, 'rgba(54, 40, 28, 0)');
  grad.addColorStop(0.35, 'rgba(54, 40, 28, 0.28)');
  grad.addColorStop(0.75, 'rgba(44, 32, 22, 0.58)');
  grad.addColorStop(1.0, 'rgba(38, 26, 18, 0.72)');
  ctx.fillStyle = grad;
  ctx.fillRect(0, 166, 256, 90);

  // Deterministic pseudo-random mud flecks and splatters
  let seed = 1337;
  const rand = () => {
    seed = (seed * 9301 + 49297) % 233280;
    return seed / 233280;
  };

  const mudColors = ['#2b1e15', '#38271a', '#473222', '#22170f'];
  for (let i = 0; i < 240; i++) {
    const x = rand() * 256;
    const y = 148 + Math.pow(rand(), 1.7) * 108;
    const rx = 0.9 + rand() * 2.6;
    const ry = rx * (0.7 + rand() * 0.7);
    const angle = rand() * Math.PI;

    ctx.fillStyle = mudColors[Math.floor(rand() * mudColors.length)];
    ctx.globalAlpha = 0.35 + rand() * 0.45;
    ctx.beginPath();
    ctx.ellipse(x, y, rx, ry, angle, 0, Math.PI * 2);
    ctx.fill();
  }
  ctx.globalAlpha = 1.0;

  const texture = new THREE.CanvasTexture(canvas);
  texture.wrapS = THREE.RepeatWrapping;
  texture.wrapT = THREE.ClampToEdgeWrapping;
  return texture;
}

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
    const mudTexture = ENABLE_MUD_SPLATTER ? createMudSplatterTexture() : null;

    // Materials
    const bodyMat = new THREE.MeshStandardMaterial({
      color: VEHICLE_PALETTE.bodyColor,
      roughness: 0.65,
      metalness: 0.12,
      flatShading: true,
      map: mudTexture,
    });

    const roofMat = new THREE.MeshStandardMaterial({
      color: VEHICLE_PALETTE.roofColor,
      roughness: 0.75,
      metalness: 0.05,
      flatShading: true,
    });

    const trimMat = new THREE.MeshStandardMaterial({
      color: VEHICLE_PALETTE.trimColor,
      roughness: 0.8,
      metalness: 0.2,
      flatShading: true,
      map: mudTexture,
    });

    const bumperMat = new THREE.MeshStandardMaterial({
      color: VEHICLE_PALETTE.bumperColor,
      roughness: 0.85,
      metalness: 0.3,
      flatShading: true,
    });

    const rackMat = new THREE.MeshStandardMaterial({
      color: VEHICLE_PALETTE.rackColor,
      roughness: 0.75,
      metalness: 0.35,
      flatShading: true,
    });

    const glassMat = new THREE.MeshStandardMaterial({
      color: VEHICLE_PALETTE.glassColor,
      roughness: 0.15,
      metalness: 0.45,
      transparent: true,
      opacity: 0.82,
      flatShading: true,
    });

    const chromeMat = new THREE.MeshStandardMaterial({
      color: 0xd6dede,
      roughness: 0.25,
      metalness: 0.85,
      flatShading: true,
    });

    const headlightMat = new THREE.MeshStandardMaterial({
      color: 0xfff0c4,
      emissive: 0xffdf88,
      emissiveIntensity: 0.7,
      roughness: 0.2,
    });

    const tailLampMat = new THREE.MeshStandardMaterial({
      color: 0xff2222,
      emissive: 0xdd1111,
      emissiveIntensity: 0.65,
      roughness: 0.3,
    });

    const indicatorMat = new THREE.MeshStandardMaterial({
      color: 0xffa012,
      emissive: 0xff8c00,
      emissiveIntensity: 0.6,
      roughness: 0.3,
    });

    const skidMat = new THREE.MeshStandardMaterial({
      color: 0x363a38,
      roughness: 0.8,
      metalness: 0.3,
      flatShading: true,
    });

    const cargoBoxMat = new THREE.MeshStandardMaterial({
      color: 0x545b4c, // Rugged olive-tan expedition cargo container
      roughness: 0.7,
      metalness: 0.1,
      flatShading: true,
    });

    const jerryCanMat = new THREE.MeshStandardMaterial({
      color: 0x8a382c, // Vintage terracotta-red stamped steel fuel can
      roughness: 0.65,
      metalness: 0.25,
      flatShading: true,
    });

    // =========================================================================
    // 1. UNDERBODY CHASSIS, LADDER FRAME, SOLID AXLES & SKID PLATES
    // Physically aligned with chassisContact.ts underbody collision points
    // =========================================================================

    // Longitudinal ladder frame rails
    const frameRails = new THREE.Mesh(new THREE.BoxGeometry(1.20, 0.16, 3.30), trimMat);
    frameRails.position.set(0, -0.52, 0);
    this.bodyMesh.add(frameRails);

    // Center transfer case / transmission skid belly pan
    const bellyPan = new THREE.Mesh(new THREE.BoxGeometry(0.90, 0.08, 1.40), skidMat);
    bellyPan.position.set(0, -0.70, 0);
    bellyPan.receiveShadow = true;
    this.bodyMesh.add(bellyPan);

    // Front & rear solid axle tubes
    const axleTubeGeom = new THREE.CylinderGeometry(0.045, 0.045, 1.45, 8);
    axleTubeGeom.rotateZ(Math.PI / 2);
    const diffGeom = new THREE.SphereGeometry(0.12, 10, 8);

    const frontAxle = new THREE.Mesh(axleTubeGeom, trimMat);
    frontAxle.position.set(0, -0.68, 1.30);
    this.bodyMesh.add(frontAxle);

    const frontDiff = new THREE.Mesh(diffGeom, skidMat);
    frontDiff.position.set(0, -0.70, 1.30);
    frontDiff.scale.set(1.0, 0.85, 1.1);
    this.bodyMesh.add(frontDiff);

    const rearAxle = new THREE.Mesh(axleTubeGeom, trimMat);
    rearAxle.position.set(0, -0.68, -1.30);
    this.bodyMesh.add(rearAxle);

    const rearDiff = new THREE.Mesh(diffGeom, skidMat);
    rearDiff.position.set(0, -0.70, -1.30);
    rearDiff.scale.set(1.0, 0.85, 1.1);
    this.bodyMesh.add(rearDiff);

    // Front & rear swept approach/departure skid plates
    const frontSkidGeom = new THREE.BoxGeometry(0.75, 0.06, 0.72);
    frontSkidGeom.rotateX(0.25);
    const frontSkid = new THREE.Mesh(frontSkidGeom, skidMat);
    frontSkid.position.set(0, -0.64, 1.70);
    this.bodyMesh.add(frontSkid);

    const rearSkidGeom = new THREE.BoxGeometry(0.75, 0.06, 0.70);
    rearSkidGeom.rotateX(-0.25);
    const rearSkid = new THREE.Mesh(rearSkidGeom, skidMat);
    rearSkid.position.set(0, -0.63, -1.68);
    this.bodyMesh.add(rearSkid);

    // =========================================================================
    // 2. MAIN BODY LOWER TUB & WHEEL WELL ARCH CUTOUTS
    // Generous wheel well openings prevent clipping at 32° steer & 0.35m jounce
    // =========================================================================

    // Central lower tub (between axles, Z = -0.85 to +0.85m)
    const centerTub = new THREE.Mesh(
      new RoundedBoxGeometry(1.72, 0.44, 1.68, 2, 0.03),
      bodyMat
    );
    centerTub.position.set(0, -0.04, 0);
    centerTub.castShadow = true;
    centerTub.receiveShadow = true;
    this.bodyMesh.add(centerTub);

    // Rocker sills along bottom sides between wheels
    const rockerLeft = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.22, 1.62), trimMat);
    rockerLeft.position.set(0.86, -0.16, 0);
    this.bodyMesh.add(rockerLeft);

    const rockerRight = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.22, 1.62), trimMat);
    rockerRight.position.set(-0.86, -0.16, 0);
    this.bodyMesh.add(rockerRight);

    // Narrowed inner wheel-well boxes (X = ±0.58m leaves 0.36m clear for each wheel)
    const frontInnerWell = new THREE.Mesh(new THREE.BoxGeometry(1.16, 0.38, 0.90), bodyMat);
    frontInnerWell.position.set(0, -0.06, 1.30);
    this.bodyMesh.add(frontInnerWell);

    const rearInnerWell = new THREE.Mesh(new THREE.BoxGeometry(1.16, 0.38, 0.86), bodyMat);
    rearInnerWell.position.set(0, -0.06, -1.30);
    this.bodyMesh.add(rearInnerWell);

    // Arched fender flare trims framing the wheel wells (arch crest at Y = +0.08m, curved profile)
    for (const zAxle of [1.30, -1.30]) {
      for (const signX of [-1, 1]) {
        // Upper arch eyebrow
        const archTop = new THREE.Mesh(new RoundedBoxGeometry(0.07, 0.08, 0.88, 2, 0.02), trimMat);
        archTop.position.set(signX * 0.90, 0.08, zAxle);
        this.bodyMesh.add(archTop);

        // Front and rear downward curved legs of the arch
        const legFwd = new THREE.Mesh(new RoundedBoxGeometry(0.07, 0.22, 0.10, 2, 0.02), trimMat);
        legFwd.position.set(signX * 0.90, -0.05, zAxle + 0.40);
        this.bodyMesh.add(legFwd);

        const legAft = new THREE.Mesh(new RoundedBoxGeometry(0.07, 0.22, 0.10, 2, 0.02), trimMat);
        legAft.position.set(signX * 0.90, -0.05, zAxle - 0.40);
        this.bodyMesh.add(legAft);
      }
    }

    // =========================================================================
    // 3. FRONT HOOD, BULGING FENDERS & CLASSIC FASCIA
    // =========================================================================

    // Flat hood with subtle chamfer
    const hood = new THREE.Mesh(
      new RoundedBoxGeometry(1.04, 0.26, 1.28, 2, 0.03),
      bodyMat
    );
    hood.position.set(0, 0.34, 1.32);
    hood.castShadow = true;
    this.bodyMesh.add(hood);

    // Subtle center hood power bulge
    const hoodBulge = new THREE.Mesh(
      new RoundedBoxGeometry(0.48, 0.03, 0.96, 2, 0.015),
      bodyMat
    );
    hoodBulge.position.set(0, 0.48, 1.30);
    this.bodyMesh.add(hoodBulge);

    // Separate bulging front fenders
    for (const signX of [-1, 1]) {
      // Top crown of fender
      const fenderCrown = new THREE.Mesh(
        new RoundedBoxGeometry(0.36, 0.25, 1.28, 2, 0.03),
        bodyMat
      );
      fenderCrown.position.set(signX * 0.70, 0.33, 1.32);
      fenderCrown.castShadow = true;
      this.bodyMesh.add(fenderCrown);

      // Fender front drop (forward of wheel well down to bumper level)
      const fenderFrontDrop = new THREE.Mesh(
        new RoundedBoxGeometry(0.36, 0.32, 0.28, 2, 0.025),
        bodyMat
      );
      fenderFrontDrop.position.set(signX * 0.70, 0.06, 1.84);
      fenderFrontDrop.castShadow = true;
      this.bodyMesh.add(fenderFrontDrop);
    }

    // Front vertical fascia panel behind grille
    const frontFascia = new THREE.Mesh(
      new RoundedBoxGeometry(1.72, 0.34, 0.06, 2, 0.02),
      bodyMat
    );
    frontFascia.position.set(0, 0.26, 1.96);
    this.bodyMesh.add(frontFascia);

    // Dark rectangular grille opening
    const grilleBox = new THREE.Mesh(new THREE.BoxGeometry(0.86, 0.24, 0.04), trimMat);
    grilleBox.position.set(0, 0.26, 1.98);
    this.bodyMesh.add(grilleBox);

    // Grille horizontal chrome/steel bars (4 bars)
    for (let b = 0; b < 4; b++) {
      const yBar = 0.17 + b * 0.06;
      const bar = new THREE.Mesh(new THREE.BoxGeometry(0.82, 0.018, 0.02), chromeMat);
      bar.position.set(0, yBar, 2.00);
      this.bodyMesh.add(bar);
    }

    // Classic round headlights with chrome bezel rings on the outer fenders
    const headlightLensGeom = new THREE.CylinderGeometry(0.14, 0.14, 0.04, 16);
    headlightLensGeom.rotateX(Math.PI / 2);

    const bezelGeom = new THREE.CylinderGeometry(0.17, 0.17, 0.03, 16);
    bezelGeom.rotateX(Math.PI / 2);

    for (const signX of [-1, 1]) {
      const bezel = new THREE.Mesh(bezelGeom, chromeMat);
      bezel.position.set(signX * 0.64, 0.28, 1.97);
      this.bodyMesh.add(bezel);

      const lens = new THREE.Mesh(headlightLensGeom, headlightMat);
      lens.position.set(signX * 0.64, 0.28, 1.985);
      this.bodyMesh.add(lens);

      // Amber corner indicator lamps
      const indicator = new THREE.Mesh(
        new RoundedBoxGeometry(0.06, 0.08, 0.04, 2, 0.01),
        indicatorMat
      );
      indicator.position.set(signX * 0.85, 0.28, 1.96);
      this.bodyMesh.add(indicator);
    }

    // Front slim dark-steel bumper bar with tow hooks
    const frontBumper = new THREE.Mesh(
      new RoundedBoxGeometry(1.88, 0.14, 0.15, 2, 0.025),
      bumperMat
    );
    frontBumper.position.set(0, 0.02, 2.08);
    frontBumper.castShadow = true;
    this.bodyMesh.add(frontBumper);

    // Front lower apron connecting down to approach skid
    const frontApron = new THREE.Mesh(new THREE.BoxGeometry(1.68, 0.28, 0.12), bumperMat);
    frontApron.position.set(0, -0.19, 2.03);
    this.bodyMesh.add(frontApron);

    // Two small steel tow hooks on front bumper
    const hookGeom = new THREE.TorusGeometry(0.035, 0.012, 6, 12);
    for (const signX of [-1, 1]) {
      const hook = new THREE.Mesh(hookGeom, chromeMat);
      hook.position.set(signX * 0.35, -0.04, 2.16);
      this.bodyMesh.add(hook);
    }

    // =========================================================================
    // 4. CABIN GREENHOUSE, DOORS, WINDOWS, MIRRORS & WHIP ANTENNA
    // =========================================================================

    // Lower cabin waist & doors (Z = -1.60 to +0.68m)
    const cabinWaist = new THREE.Mesh(
      new RoundedBoxGeometry(1.66, 0.36, 2.30, 2, 0.03),
      bodyMat
    );
    cabinWaist.position.set(0, 0.38, -0.45);
    cabinWaist.castShadow = true;
    this.bodyMesh.add(cabinWaist);

    // Upper cabin greenhouse pillars & frame
    const cabinGreenhouse = new THREE.Mesh(
      new RoundedBoxGeometry(1.58, 0.72, 2.20, 2, 0.04),
      roofMat
    );
    cabinGreenhouse.position.set(0, 0.92, -0.45);
    cabinGreenhouse.castShadow = true;
    cabinGreenhouse.receiveShadow = true;
    this.bodyMesh.add(cabinGreenhouse);

    // Windshield (upright, near-vertical ~7° rake)
    const windshield = new THREE.Mesh(
      new RoundedBoxGeometry(1.44, 0.52, 0.04, 2, 0.02),
      glassMat
    );
    windshield.position.set(0, 0.94, 0.655);
    windshield.rotation.x = -0.12; // ~6.9° rake
    this.bodyMesh.add(windshield);

    // Side windows (large square door windows with slightly rounded corners)
    for (const signX of [-1, 1]) {
      // Front door window
      const winFront = new THREE.Mesh(
        new RoundedBoxGeometry(0.03, 0.46, 0.62, 2, 0.02),
        glassMat
      );
      winFront.position.set(signX * 0.795, 0.94, 0.18);
      this.bodyMesh.add(winFront);

      // Rear passenger window
      const winMid = new THREE.Mesh(
        new RoundedBoxGeometry(0.03, 0.46, 0.58, 2, 0.02),
        glassMat
      );
      winMid.position.set(signX * 0.795, 0.94, -0.50);
      this.bodyMesh.add(winMid);

      // Rear cargo window
      const winRear = new THREE.Mesh(
        new RoundedBoxGeometry(0.03, 0.46, 0.50, 2, 0.02),
        glassMat
      );
      winRear.position.set(signX * 0.795, 0.94, -1.12);
      this.bodyMesh.add(winRear);

      // Door handles
      const handleFront = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.03, 0.11), trimMat);
      handleFront.position.set(signX * 0.84, 0.36, 0.22);
      this.bodyMesh.add(handleFront);

      const handleRear = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.03, 0.11), trimMat);
      handleRear.position.set(signX * 0.84, 0.36, -0.42);
      this.bodyMesh.add(handleRear);

      // A-Pillar Mirrors on short arms (sticks out 0.11m ≤ 0.12m limit)
      const mirrorArm = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.11, 6), trimMat);
      mirrorArm.rotateZ(signX * (Math.PI / 2));
      mirrorArm.position.set(signX * 0.85, 0.82, 0.58);
      this.bodyMesh.add(mirrorArm);

      const mirrorHead = new THREE.Mesh(
        new RoundedBoxGeometry(0.03, 0.16, 0.11, 2, 0.02),
        trimMat
      );
      mirrorHead.position.set(signX * 0.91, 0.82, 0.58);
      this.bodyMesh.add(mirrorHead);

      const mirrorFace = new THREE.Mesh(new THREE.BoxGeometry(0.01, 0.14, 0.09), chromeMat);
      mirrorFace.position.set(signX * 0.90, 0.82, 0.57);
      this.bodyMesh.add(mirrorFace);
    }

    // Rear tailgate window
    const rearGlass = new THREE.Mesh(
      new RoundedBoxGeometry(1.40, 0.46, 0.04, 2, 0.02),
      glassMat
    );
    rearGlass.position.set(0, 0.94, -1.555);
    this.bodyMesh.add(rearGlass);

    // Thin whip antenna on front-left fender (~0.9m long, tilted 6° back)
    const antennaGeom = new THREE.CylinderGeometry(0.004, 0.007, 0.90, 6);
    antennaGeom.rotateX(-0.10);
    const antenna = new THREE.Mesh(antennaGeom, chromeMat);
    antenna.position.set(-0.76, 0.90, 1.25);
    this.bodyMesh.add(antenna);

    // =========================================================================
    // 5. TWO-TONE ROOF PANEL WITH DRIP-RAIL OVERHANG & LOW-PROFILE ROOF RACK
    // Overhangs cabin by ~3-4 cm on all sides; total height stays ≤ 1.46m
    // =========================================================================

    // Overhanging roof panel (width 1.66m vs cabin 1.58m => 4cm drip rail overhang)
    const dripRoof = new THREE.Mesh(
      new RoundedBoxGeometry(1.66, 0.08, 2.28, 2, 0.025),
      roofMat
    );
    dripRoof.position.set(0, 1.32, -0.45);
    dripRoof.castShadow = true;
    this.bodyMesh.add(dripRoof);

    // Subtle roof center cap / ribs
    const roofCrown = new THREE.Mesh(
      new RoundedBoxGeometry(1.54, 0.05, 2.14, 2, 0.02),
      roofMat
    );
    roofCrown.position.set(0, 1.37, -0.45);
    this.bodyMesh.add(roofCrown);

    // Low-profile roof-rack basket frame (rails at Y = 1.44m, X = ±0.70m, Z = -1.40 to +0.50m)
    const rackSideL = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.045, 1.90), rackMat);
    rackSideL.position.set(-0.68, 1.43, -0.45);
    this.bodyMesh.add(rackSideL);

    const rackSideR = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.045, 1.90), rackMat);
    rackSideR.position.set(0.68, 1.43, -0.45);
    this.bodyMesh.add(rackSideR);

    const rackEndFwd = new THREE.Mesh(new THREE.BoxGeometry(1.38, 0.045, 0.03), rackMat);
    rackEndFwd.position.set(0, 1.43, 0.50);
    this.bodyMesh.add(rackEndFwd);

    const rackEndAft = new THREE.Mesh(new THREE.BoxGeometry(1.38, 0.045, 0.03), rackMat);
    rackEndAft.position.set(0, 1.43, -1.40);
    this.bodyMesh.add(rackEndAft);

    // Roof rack grid floor crossbars (5 thin crossbars)
    for (let c = 0; c < 5; c++) {
      const zBar = -1.15 + c * 0.38;
      const bar = new THREE.Mesh(new THREE.BoxGeometry(1.34, 0.02, 0.025), rackMat);
      bar.position.set(0, 1.415, zBar);
      this.bodyMesh.add(bar);
    }

    // Cargo Box on roof rack
    const cargoBox = new THREE.Mesh(
      new RoundedBoxGeometry(0.52, 0.08, 0.76, 2, 0.02),
      cargoBoxMat
    );
    cargoBox.position.set(-0.25, 1.445, -0.80);
    cargoBox.castShadow = true;
    this.bodyMesh.add(cargoBox);

    // Jerry Can on roof rack
    const jerryCan = new THREE.Mesh(
      new RoundedBoxGeometry(0.24, 0.08, 0.40, 2, 0.02),
      jerryCanMat
    );
    jerryCan.position.set(0.32, 1.445, -0.20);
    jerryCan.castShadow = true;
    this.bodyMesh.add(jerryCan);

    // Thin cargo tie-down straps over cargo
    const strapBox = new THREE.Mesh(new THREE.BoxGeometry(0.54, 0.015, 0.03), trimMat);
    strapBox.position.set(-0.25, 1.455, -0.80);
    this.bodyMesh.add(strapBox);

    const strapCan = new THREE.Mesh(new THREE.BoxGeometry(0.26, 0.015, 0.03), trimMat);
    strapCan.position.set(0.32, 1.455, -0.20);
    this.bodyMesh.add(strapCan);

    // =========================================================================
    // 6. REAR END: TAILGATE, SPARE WHEEL, REAR BUMPER & RED TAIL LAMPS
    // =========================================================================

    // Rear bumper bar
    const rearBumper = new THREE.Mesh(
      new RoundedBoxGeometry(1.88, 0.14, 0.15, 2, 0.025),
      bumperMat
    );
    rearBumper.position.set(0, 0.02, -1.94);
    rearBumper.castShadow = true;
    this.bodyMesh.add(rearBumper);

    // Rear apron down to departure skid
    const rearApron = new THREE.Mesh(new THREE.BoxGeometry(1.68, 0.28, 0.12), bumperMat);
    rearApron.position.set(0, -0.19, -1.90);
    this.bodyMesh.add(rearApron);

    // Small red tail lamps on rear corners
    for (const signX of [-1, 1]) {
      const tailLamp = new THREE.Mesh(
        new RoundedBoxGeometry(0.08, 0.14, 0.04, 2, 0.015),
        tailLampMat
      );
      tailLamp.position.set(signX * 0.78, 0.24, -1.82);
      this.bodyMesh.add(tailLamp);
    }

    // Full-sized spare wheel on rear tailgate (radius 0.36m, matching wheel style)
    const spareTireGeom = new THREE.CylinderGeometry(0.36, 0.36, 0.18, 16);
    spareTireGeom.rotateX(Math.PI / 2);
    const spareWheel = new THREE.Mesh(spareTireGeom, trimMat);
    spareWheel.position.set(0, 0.45, -1.96);
    spareWheel.castShadow = true;
    this.bodyMesh.add(spareWheel);

    const spareRimGeom = new THREE.CylinderGeometry(0.22, 0.22, 0.19, 12);
    spareRimGeom.rotateX(Math.PI / 2);
    const spareRim = new THREE.Mesh(spareRimGeom, chromeMat);
    spareRim.position.set(0, 0.45, -1.96);
    this.bodyMesh.add(spareRim);
  }

  private buildWheels(): void {
    const tireMat = new THREE.MeshStandardMaterial({
      color: 0x202221,
      roughness: 0.9,
      metalness: 0.08,
      flatShading: true,
    });

    const rimMat = new THREE.MeshStandardMaterial({
      color: 0xdfdad0, // Light-grey vintage steel rim
      roughness: 0.55,
      metalness: 0.35,
      flatShading: true,
    });

    const spokeMat = new THREE.MeshStandardMaterial({
      color: 0xd97736, // Contrast terracotta-orange spin marker spokes
      roughness: 0.4,
      metalness: 0.1,
      flatShading: true,
    });

    const lugMat = new THREE.MeshStandardMaterial({
      color: 0xf5f7f8,
      roughness: 0.3,
      metalness: 0.65,
    });

    const treadLugGeom = new THREE.BoxGeometry(0.24, 0.035, 0.08);

    for (let i = 0; i < 4; i++) {
      const steerGroup = new THREE.Group();
      const spinGroup = new THREE.Group();
      steerGroup.add(spinGroup);

      // Main tire cylinder (radius 0.38m, width 0.28m)
      const tireGeom = new THREE.CylinderGeometry(0.38, 0.38, 0.28, 16);
      tireGeom.rotateZ(Math.PI / 2);
      const tireMesh = new THREE.Mesh(tireGeom, tireMat);
      tireMesh.castShadow = true;
      spinGroup.add(tireMesh);

      // Chunky all-terrain tread blocks around the circumference (12 perimeter lugs)
      for (let t = 0; t < 12; t++) {
        const theta = (t * Math.PI) / 6;
        const treadBlock = new THREE.Mesh(treadLugGeom, tireMat);
        treadBlock.position.set(0, Math.cos(theta) * 0.385, Math.sin(theta) * 0.385);
        treadBlock.rotation.x = -theta;
        spinGroup.add(treadBlock);
      }

      // Light-grey steel rim (radius 0.24m, width 0.285m)
      const rimGeom = new THREE.CylinderGeometry(0.24, 0.24, 0.285, 14);
      rimGeom.rotateZ(Math.PI / 2);
      const rimMesh = new THREE.Mesh(rimGeom, rimMat);
      spinGroup.add(rimMesh);

      const isLeft = i === 0 || i === 2; // FL (0) and RL (2)
      const faceOffset = isLeft ? 0.145 : -0.145;

      // High-contrast spin marker spokes so wheel rotation remains clearly readable
      const spoke1 = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.38, 0.03), spokeMat);
      spoke1.position.x = faceOffset;
      spinGroup.add(spoke1);

      const spoke2 = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.38, 0.03), spokeMat);
      spoke2.position.x = faceOffset;
      spoke2.rotation.x = Math.PI / 2;
      spinGroup.add(spoke2);

      // Central hub nut
      const hubNutGeom = new THREE.CylinderGeometry(0.06, 0.06, 0.04, 6);
      hubNutGeom.rotateZ(Math.PI / 2);
      const hubNut = new THREE.Mesh(hubNutGeom, lugMat);
      hubNut.position.x = faceOffset * 1.05;
      spinGroup.add(hubNut);

      // 5 Lug nuts arranged in a ring on the steel wheel face
      for (let n = 0; n < 5; n++) {
        const nutAngle = (n * Math.PI * 2) / 5;
        const lug = new THREE.Mesh(new THREE.BoxGeometry(0.02, 0.025, 0.025), lugMat);
        lug.position.set(
          faceOffset * 1.02,
          Math.cos(nutAngle) * 0.12,
          Math.sin(nutAngle) * 0.12
        );
        spinGroup.add(lug);
      }

      this.group.add(steerGroup);
      this.wheelSteerGroups.push(steerGroup);
      this.wheelSpinGroups.push(spinGroup);
    }
  }

  /**
   * Updates visual mesh transforms from rigid body, suspension, and wheel rotational dynamics.
   * Signature and implementation preserved exactly for zero per-frame allocations.
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
