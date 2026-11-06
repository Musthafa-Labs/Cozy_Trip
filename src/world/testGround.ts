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

export interface ObstacleContactResult {
  penetration: number;
  normal: THREE.Vector3;
  surface: SurfaceType;
  pointOnObstacle: THREE.Vector3;
}

export interface TestStation {
  id: string;
  name: string;
  zone: string;
  description: string;
  position: THREE.Vector3;
  headingRad: number;
}

export interface StaticBoxObstacle {
  id: string;
  name: string;
  center: THREE.Vector3;
  halfExtents: THREE.Vector3; // halfWidth (X), halfHeight (Y), halfLength (Z)
  yawRad: number;
  surface: SurfaceType;
}

export class TestGround {
  public readonly mesh: THREE.Group;
  private readonly defaultNormal = new THREE.Vector3(0, 1, 0);

  // Exact 1.2m x 1.2m x 1.2m static solid cube positioned directly in driving path at Z = 65m
  public readonly staticObstacles: StaticBoxObstacle[] = [
    {
      id: 'target_cube',
      name: 'Solid 1.2m Cube',
      center: new THREE.Vector3(0, 0.60, 65.0),
      halfExtents: new THREE.Vector3(0.60, 0.60, 0.60),
      yawRad: 0,
      surface: 'asphalt',
    },
  ];

  // Single test station: vehicle on flat road, facing directly toward the cube
  public readonly stations: TestStation[] = [
    {
      id: 'cube_collision_test',
      name: 'Solid Cube Collision Test',
      zone: 'Testing Arena',
      description: 'Single straight road with one solid 1.2m cube at Z = 65m. Approach and impact at 10, 30 or 50 km/h.',
      position: new THREE.Vector3(0, 1.06, 0),
      headingRad: 0,
    },
  ];

  // Road geometry dimensions: 14m wide, 200m long (-20m to +180m)
  public static readonly ROAD_WIDTH = 14.0;
  public static readonly ROAD_HALF_WIDTH = 7.0;
  public static readonly ROAD_START_Z = -20.0;
  public static readonly ROAD_END_Z = 180.0;

  constructor() {
    this.mesh = new THREE.Group();
    this.buildScene();
  }

  /**
   * Fast, exact analytical ground query for level asphalt road.
   * Everywhere on road: height is strictly 0.0, normal is strictly (0, 1, 0).
   */
  public getHeightNormalSurface(x: number, z: number): GroundQuery {
    const onRoad = Math.abs(x) <= TestGround.ROAD_HALF_WIDTH &&
                   z >= TestGround.ROAD_START_Z &&
                   z <= TestGround.ROAD_END_Z;

    return {
      height: 0.0,
      normal: this.defaultNormal,
      surface: onRoad ? 'asphalt' : 'dirt',
    };
  }

  /**
   * Evaluates exact 3D contact penetration and outward face normal against the solid cube.
   * Outputs the outward unit normal and penetration depth along the closest face.
   */
  public getObstacleContact(
    worldPoint: THREE.Vector3,
    outResult: ObstacleContactResult
  ): boolean {
    const obs = this.staticObstacles[0];

    // Distance to obstacle center
    const dx = worldPoint.x - obs.center.x;
    const dy = worldPoint.y - obs.center.y;
    const dz = worldPoint.z - obs.center.z;

    // Check against box bounds (with rotation if any)
    const cosY = Math.cos(-obs.yawRad);
    const sinY = Math.sin(-obs.yawRad);
    const localX = cosY * dx - sinY * dz;
    const localZ = sinY * dx + cosY * dz;
    const localY = dy;

    const distX = obs.halfExtents.x - Math.abs(localX);
    const distY = obs.halfExtents.y - Math.abs(localY);
    const distZ = obs.halfExtents.z - Math.abs(localZ);

    // If point is strictly inside the 3D cube volume
    if (distX > 0 && distY > 0 && distZ > 0) {
      // Find the closest face (smallest penetration distance)
      let minPen = distX;
      let localNormX = Math.sign(localX);
      let localNormY = 0;
      let localNormZ = 0;

      if (distZ < minPen) {
        minPen = distZ;
        localNormX = 0;
        localNormY = 0;
        localNormZ = Math.sign(localZ);
      }

      if (distY < minPen) {
        minPen = distY;
        localNormX = 0;
        localNormY = Math.sign(localY);
        localNormZ = 0;
      }

      // Transform outward local normal back to world space
      const cosYaw = Math.cos(obs.yawRad);
      const sinYaw = Math.sin(obs.yawRad);
      const worldNormX = cosYaw * localNormX - sinYaw * localNormZ;
      const worldNormZ = sinYaw * localNormX + cosYaw * localNormZ;

      outResult.penetration = minPen;
      outResult.normal.set(worldNormX, localNormY, worldNormZ).normalize();
      outResult.surface = obs.surface;
      outResult.pointOnObstacle.copy(worldPoint).addScaledVector(outResult.normal, minPen);
      return true;
    }

    return false;
  }

  /**
   * Constructs visual meshes: one straight asphalt road and one plain grey cube.
   * Zero decorations, zero stripes, zero clutter.
   */
  private buildScene(): void {
    // 1. Surrounding flat ground
    const groundGeom = new THREE.PlaneGeometry(300, 300, 1, 1);
    groundGeom.rotateX(-Math.PI / 2);
    const groundMat = new THREE.MeshStandardMaterial({
      color: 0xc4beaf, // Muted sandy ground
      roughness: 0.95,
      metalness: 0.02,
    });
    const groundMesh = new THREE.Mesh(groundGeom, groundMat);
    groundMesh.position.set(0, -0.01, 80);
    groundMesh.receiveShadow = true;
    this.mesh.add(groundMesh);

    // 2. Straight Asphalt Road (14m wide, 200m long: Z = -20 to 180)
    const roadLength = TestGround.ROAD_END_Z - TestGround.ROAD_START_Z;
    const roadCenterZ = (TestGround.ROAD_START_Z + TestGround.ROAD_END_Z) * 0.5;

    const roadGeom = new THREE.PlaneGeometry(TestGround.ROAD_WIDTH, roadLength, 1, 1);
    roadGeom.rotateX(-Math.PI / 2);
    const roadMat = new THREE.MeshStandardMaterial({
      color: 0x383b40, // Flat asphalt
      roughness: 0.85,
      metalness: 0.08,
    });
    const roadMesh = new THREE.Mesh(roadGeom, roadMat);
    roadMesh.position.set(0, 0.001, roadCenterZ);
    roadMesh.receiveShadow = true;
    this.mesh.add(roadMesh);

    // Subtle edge lines to define the 14m road width
    const edgeLineWidth = 0.25;
    const edgeGeom = new THREE.PlaneGeometry(edgeLineWidth, roadLength, 1, 1);
    edgeGeom.rotateX(-Math.PI / 2);
    const edgeMat = new THREE.MeshStandardMaterial({
      color: 0xdcd8cd,
      roughness: 0.7,
    });
    const edgeL = new THREE.Mesh(edgeGeom, edgeMat);
    edgeL.position.set(-TestGround.ROAD_HALF_WIDTH + edgeLineWidth * 0.5, 0.002, roadCenterZ);
    edgeL.receiveShadow = true;
    this.mesh.add(edgeL);

    const edgeR = new THREE.Mesh(edgeGeom, edgeMat);
    edgeR.position.set(TestGround.ROAD_HALF_WIDTH - edgeLineWidth * 0.5, 0.002, roadCenterZ);
    edgeR.receiveShadow = true;
    this.mesh.add(edgeR);

    // 3. Exactly one plain solid 1.2m cube at (0, 0.6, 65)
    // Plain grey material, no stripes, no textures, no decorative details
    const obs = this.staticObstacles[0];
    const cubeGeom = new THREE.BoxGeometry(
      obs.halfExtents.x * 2,
      obs.halfExtents.y * 2,
      obs.halfExtents.z * 2
    );
    const cubeMat = new THREE.MeshStandardMaterial({
      color: 0x7a7e85, // Plain neutral concrete grey
      roughness: 0.8,
      metalness: 0.1,
    });
    const cubeMesh = new THREE.Mesh(cubeGeom, cubeMat);
    cubeMesh.position.copy(obs.center);
    cubeMesh.castShadow = true;
    cubeMesh.receiveShadow = true;
    this.mesh.add(cubeMesh);
  }
}
