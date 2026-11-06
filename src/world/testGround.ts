/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import * as THREE from 'three';
import { SurfaceType } from '../vehicle/tire';

export interface GroundQuery {
  height: number;
  normal: THREE.Vector3;
  surface: SurfaceType;
}

export interface TestStation {
  name: string;
  description: string;
  position: THREE.Vector3;
  headingRad: number;
}

export class TestGround {
  public readonly mesh: THREE.Group;
  private readonly cachedNormal = new THREE.Vector3(0, 1, 0);

  // Single clean straight test road definition
  public readonly stations: TestStation[] = [
    {
      name: 'High-Speed Straight Test Track',
      description: '5 km perfectly flat, straight asphalt road with clearly marked edges and distance signs. Zero obstacles.',
      position: new THREE.Vector3(0, 1.06, 0),
      headingRad: 0, // Facing straight forward along +Z
    },
  ];

  // Road geometry dimensions
  public static readonly ROAD_WIDTH = 14.0;       // 14 m total asphalt width (2 wide lanes)
  public static readonly HALF_WIDTH = 7.0;
  public static readonly ROAD_START_Z = -300.0;   // 300 m behind spawn
  public static readonly ROAD_END_Z = 4500.0;     // 4.5 km forward
  public static readonly ROAD_LENGTH = 4800.0;

  constructor() {
    this.mesh = new THREE.Group();
    this.buildCleanStraightRoad();
  }

  /**
   * Fast, exact analytical ground query for perfectly flat terrain.
   * On road (|x| <= 7.0 m): surface is high-grip asphalt.
   * Off road (|x| > 7.0 m): surface is medium-grip dirt shoulder.
   * Height is strictly 0.0, normal is strictly (0, 1, 0).
   */
  public getHeightNormalSurface(x: number, _z: number): GroundQuery {
    const isAsphalt = Math.abs(x) <= TestGround.HALF_WIDTH;
    return {
      height: 0.0,
      normal: this.cachedNormal,
      surface: isAsphalt ? 'asphalt' : 'dirt',
    };
  }

  /**
   * Builds a clean, minimal, long straight road with edge markings and subtle distance hashes.
   * Zero obstacles, zero trees, zero rocks, zero ramps, zero clutter.
   */
  private buildCleanStraightRoad(): void {
    const roadZCenter = (TestGround.ROAD_START_Z + TestGround.ROAD_END_Z) * 0.5;
    const roadLength = TestGround.ROAD_LENGTH;

    // 1. Broad surrounding flat landscape ground plane (earth / calm terrain tone)
    const groundGeom = new THREE.PlaneGeometry(1200, 6000, 1, 1);
    groundGeom.rotateX(-Math.PI / 2);
    const groundMat = new THREE.MeshStandardMaterial({
      color: 0xc8c3b4, // Calm, warm muted dirt / desert plain
      roughness: 0.95,
      metalness: 0.02,
      flatShading: true,
    });
    const groundMesh = new THREE.Mesh(groundGeom, groundMat);
    groundMesh.position.set(0, -0.015, roadZCenter);
    groundMesh.receiveShadow = true;
    this.mesh.add(groundMesh);

    // 2. Main Asphalt Road Surface (Clean dark slate asphalt)
    const roadGeom = new THREE.PlaneGeometry(TestGround.ROAD_WIDTH, roadLength, 1, 1);
    roadGeom.rotateX(-Math.PI / 2);
    const roadMat = new THREE.MeshStandardMaterial({
      color: 0x36393d, // Dark matte asphalt
      roughness: 0.82,
      metalness: 0.08,
      flatShading: true,
    });
    const roadMesh = new THREE.Mesh(roadGeom, roadMat);
    roadMesh.position.set(0, 0.001, roadZCenter);
    roadMesh.receiveShadow = true;
    this.mesh.add(roadMesh);

    // 3. Crisp Outer Edge Strips (Painted white road boundary)
    const edgeLineWidth = 0.35;
    const edgeLineGeom = new THREE.PlaneGeometry(edgeLineWidth, roadLength, 1, 1);
    edgeLineGeom.rotateX(-Math.PI / 2);
    const edgeLineMat = new THREE.MeshStandardMaterial({
      color: 0xeeece5, // Warm off-white paint
      roughness: 0.65,
      metalness: 0.05,
    });

    // Left edge line
    const leftEdge = new THREE.Mesh(edgeLineGeom, edgeLineMat);
    leftEdge.position.set(-TestGround.HALF_WIDTH + edgeLineWidth * 0.5 + 0.1, 0.003, roadZCenter);
    leftEdge.receiveShadow = true;
    this.mesh.add(leftEdge);

    // Right edge line
    const rightEdge = new THREE.Mesh(edgeLineGeom, edgeLineMat);
    rightEdge.position.set(TestGround.HALF_WIDTH - edgeLineWidth * 0.5 - 0.1, 0.003, roadZCenter);
    rightEdge.receiveShadow = true;
    this.mesh.add(rightEdge);

    // 4. Broken Center Lane Divider (Dashed white line along X = 0)
    // 4m painted dash, 6m gap = 10m cycle
    const dashLength = 4.0;
    const dashWidth = 0.22;
    const dashSpacing = 10.0;
    const numDashes = Math.floor(roadLength / dashSpacing);

    const dashGeom = new THREE.PlaneGeometry(dashWidth, dashLength, 1, 1);
    dashGeom.rotateX(-Math.PI / 2);
    const dashMat = new THREE.MeshStandardMaterial({
      color: 0xeeece5,
      roughness: 0.65,
      metalness: 0.05,
    });

    const instancedDashes = new THREE.InstancedMesh(dashGeom, dashMat, numDashes);
    const dummy = new THREE.Object3D();
    for (let i = 0; i < numDashes; i++) {
      const zPos = TestGround.ROAD_START_Z + i * dashSpacing;
      dummy.position.set(0, 0.004, zPos);
      dummy.updateMatrix();
      instancedDashes.setMatrixAt(i, dummy.matrix);
    }
    instancedDashes.instanceMatrix.needsUpdate = true;
    instancedDashes.receiveShadow = true;
    this.mesh.add(instancedDashes);

    // 5. Clean, low-poly distance marker posts every 100 meters along road right side
    // Gives tactile depth cues and motion sensation without visual clutter
    const numMarkers = Math.floor((TestGround.ROAD_END_Z - 0) / 100);
    const postGeom = new THREE.BoxGeometry(0.12, 0.7, 0.12);
    const postMat = new THREE.MeshStandardMaterial({
      color: 0xded8cb,
      roughness: 0.7,
      metalness: 0.1,
    });
    const instancedPosts = new THREE.InstancedMesh(postGeom, postMat, numMarkers);

    for (let i = 0; i < numMarkers; i++) {
      const zPos = (i + 1) * 100;
      dummy.position.set(TestGround.HALF_WIDTH + 0.6, 0.35, zPos);
      dummy.scale.set(1, 1, 1);
      dummy.updateMatrix();
      instancedPosts.setMatrixAt(i, dummy.matrix);
    }
    instancedPosts.instanceMatrix.needsUpdate = true;
    instancedPosts.castShadow = true;
    this.mesh.add(instancedPosts);

    // 6. Start Grid Box at Z = 0
    const startBoxGeom = new THREE.PlaneGeometry(TestGround.ROAD_WIDTH, 1.2, 1, 1);
    startBoxGeom.rotateX(-Math.PI / 2);
    const startBoxMat = new THREE.MeshStandardMaterial({
      color: 0xe5e1d5,
      roughness: 0.8,
    });
    const startLine = new THREE.Mesh(startBoxGeom, startBoxMat);
    startLine.position.set(0, 0.002, 0);
    this.mesh.add(startLine);
  }
}
