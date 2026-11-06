/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import { clamp } from '../utils/math';

export interface SteeringConfig {
  wheelbase: number;          // m (2.6m)
  trackWidth: number;         // m (1.6m)
  maxSteerStandstill: number; // radians (32 deg = 0.5585 rad)
  minSteerHighSpeed: number;  // radians (12 deg = 0.2094 rad)
  speedFalloff: number;       // m/s (approx 7.0 m/s ~ 25 km/h for smooth decay)
  steerRate: number;          // rad/s (steer speed: approx 2.6 rad/s ~ 150 deg/s)
  selfCenterRate: number;     // rad/s (speed to return to 0 when released)
}

export class SteeringSystem {
  public readonly config: SteeringConfig;

  // Real-time state (SINGLE SOURCE OF TRUTH: positive = LEFT, negative = RIGHT)
  public currentVirtualAngle: number = 0; // Center virtual rack angle (rad, positive = LEFT)
  public steerAngleFL: number = 0;        // Ackermann inner/outer angle FL (rad, positive = LEFT)
  public steerAngleFR: number = 0;        // Ackermann inner/outer angle FR (rad, positive = LEFT)
  public steerAngleRL: number = 0;        // 0 (Rear wheels don't steer)
  public steerAngleRR: number = 0;        // 0

  constructor(config: Partial<SteeringConfig> = {}) {
    this.config = {
      wheelbase: config.wheelbase ?? 2.6,
      trackWidth: config.trackWidth ?? 1.6,
      maxSteerStandstill: config.maxSteerStandstill ?? (32 * Math.PI) / 180,
      minSteerHighSpeed: config.minSteerHighSpeed ?? (12 * Math.PI) / 180,
      speedFalloff: config.speedFalloff ?? 7.0,
      steerRate: config.steerRate ?? 2.6,
      selfCenterRate: config.selfCenterRate ?? 3.5,
    };
  }

  /**
   * REQUIRED CONVENTION:
   * Input: A / Left gives positive steer command (+1), D / Right gives negative (-1).
   * Positive steerAngle = steering LEFT as seen by driver looking forward.
   * Ackermann:
   * When turning LEFT (steer > 0): FL is inner wheel (larger positive angle) than FR.
   * When turning RIGHT (steer < 0): FR is inner wheel (larger negative magnitude) than FL.
   */
  public update(steerInput: number, vehicleSpeedMs: number, dt: number): void {
    const { wheelbase, trackWidth, maxSteerStandstill, minSteerHighSpeed, speedFalloff, steerRate, selfCenterRate } =
      this.config;

    // 1. Speed-sensitive steering angle limit
    const speedRatio = Math.exp(-Math.abs(vehicleSpeedMs) / speedFalloff);
    const dynamicMaxSteer = minSteerHighSpeed + (maxSteerStandstill - minSteerHighSpeed) * speedRatio;

    const targetAngle = steerInput * dynamicMaxSteer;

    // 2. Smooth rate limiting and self-centering
    if (Math.abs(steerInput) > 0.01) {
      const delta = targetAngle - this.currentVirtualAngle;
      const step = Math.sign(delta) * Math.min(Math.abs(delta), steerRate * dt);
      this.currentVirtualAngle += step;
    } else {
      const step = Math.sign(-this.currentVirtualAngle) * Math.min(Math.abs(this.currentVirtualAngle), selfCenterRate * dt);
      this.currentVirtualAngle += step;
    }

    this.currentVirtualAngle = clamp(this.currentVirtualAngle, -dynamicMaxSteer, dynamicMaxSteer);

    // 3. Ackermann Geometry
    const absAngle = Math.abs(this.currentVirtualAngle);

    if (absAngle < 0.001) {
      this.steerAngleFL = 0;
      this.steerAngleFR = 0;
    } else {
      const R = wheelbase / Math.tan(absAngle);
      const halfTrack = trackWidth * 0.5;

      const innerAngle = Math.atan(wheelbase / (R - halfTrack));
      const outerAngle = Math.atan(wheelbase / (R + halfTrack));

      if (this.currentVirtualAngle > 0) {
        // Turning LEFT (positive steerAngle):
        // FL (left wheel) is INNER wheel => larger positive angle
        // FR (right wheel) is OUTER wheel => smaller positive angle
        this.steerAngleFL = innerAngle;
        this.steerAngleFR = outerAngle;
      } else {
        // Turning RIGHT (negative steerAngle):
        // FR (right wheel) is INNER wheel => larger negative magnitude
        // FL (left wheel) is OUTER wheel => smaller negative magnitude
        this.steerAngleFL = -outerAngle;
        this.steerAngleFR = -innerAngle;
      }
    }

    this.steerAngleRL = 0;
    this.steerAngleRR = 0;
  }
}
