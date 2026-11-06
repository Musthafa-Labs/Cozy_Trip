/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import * as THREE from 'three';

// Reusable scratch vectors to guarantee ZERO per-frame garbage collection
export const SCRATCH_VEC3_A = new THREE.Vector3();
export const SCRATCH_VEC3_B = new THREE.Vector3();
export const SCRATCH_VEC3_C = new THREE.Vector3();
export const SCRATCH_VEC3_D = new THREE.Vector3();
export const SCRATCH_VEC3_E = new THREE.Vector3();

export const SCRATCH_QUAT_A = new THREE.Quaternion();
export const SCRATCH_QUAT_B = new THREE.Quaternion();

export const SCRATCH_MAT3_A = new THREE.Matrix3();
export const SCRATCH_MAT3_B = new THREE.Matrix3();
export const SCRATCH_MAT4_A = new THREE.Matrix4();

export function clamp(val: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, val));
}

export function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}

export function isFiniteNumber(val: number): boolean {
  return Number.isFinite(val) && !Number.isNaN(val);
}
