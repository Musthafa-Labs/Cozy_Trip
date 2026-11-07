/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import { clamp, lerp } from '../utils/math';
import { WheelDynamics } from './wheel';

export interface BrakeConfig {
  maxBrakeTorque: number;    // Total system brake torque at 100% pedal in N*m (default: 6000)
  frontBias: number;         // Front axle brake bias fraction [0 .. 1.0] (default: 0.65 = 65% front)
  handbrakeTorque: number;   // Parking / handbrake torque to rear wheels in N*m (default: 2600)
  pedalResponseTau: number;  // Brake pedal hydraulic build-up filter constant in seconds (default: 0.05)
  absEnabled: boolean;       // Anti-lock braking system toggle
  absMinSpeedMs: number;     // Minimum speed for ABS engagement in m/s (default: 2.2 m/s = 8 km/h)
  absSlipThreshold: number;  // Longitudinal slip ratio threshold to trigger pressure dump (default: 0.16)
  absPulseFrequency: number; // ABS valve cycling frequency in Hz (default: 18)
}

export const DEFAULT_BRAKE_CONFIG: BrakeConfig = {
  maxBrakeTorque: 6000,
  frontBias: 0.65,
  handbrakeTorque: 2600,
  pedalResponseTau: 0.05,
  absEnabled: false,
  absMinSpeedMs: 2.2,
  absSlipThreshold: 0.16,
  absPulseFrequency: 18,
};

export class BrakeSystem {
  public readonly config: BrakeConfig;

  // State
  public pedalPosition: number = 0; // Filtered brake pedal [0 .. 1.0]
  public isHandbrakeActive: boolean = false;
  public absActive: [boolean, boolean, boolean, boolean] = [false, false, false, false]; // [FL, FR, RL, RR]
  public absGlobalActive: boolean = false;

  // Individual wheel output brake torques in N*m
  public wheelBrakeTorques: [number, number, number, number] = [0, 0, 0, 0];

  // Internal timer for ABS valve pulsing cycle
  private absTimers: [number, number, number, number] = [0, 0, 0, 0];

  constructor(config: Partial<BrakeConfig> = {}) {
    this.config = { ...DEFAULT_BRAKE_CONFIG, ...config };
  }

  public reset(): void {
    this.pedalPosition = 0;
    this.isHandbrakeActive = false;
    this.absActive = [false, false, false, false];
    this.absGlobalActive = false;
    this.wheelBrakeTorques = [0, 0, 0, 0];
    this.absTimers = [0, 0, 0, 0];
  }

  /**
   * Updates brake pedal hydraulics, ABS slip modulation, and outputs wheel brake torques.
   *
   * @param rawBrakeInput Player brake pedal input [0 .. 1.0]
   * @param handbrakeInput Handbrake button state [true / false]
   * @param wheelsDyn Array of 4 wheel dynamics states [FL, FR, RL, RR]
   * @param vehicleSpeedMs Current vehicle ground speed in m/s
   * @param dt Fixed physics timestep in seconds (1/120)
   */
  public update(
    rawBrakeInput: number,
    handbrakeInput: boolean,
    wheelsDyn: WheelDynamics[],
    vehicleSpeedMs: number,
    dt: number
  ): void {
    // 1. Hydraulic pedal pressure build-up
    const clampedInput = clamp(rawBrakeInput, 0, 1.0);
    const alpha = clamp(dt / (this.config.pedalResponseTau + dt), 0.05, 1.0);
    this.pedalPosition = lerp(this.pedalPosition, clampedInput, alpha);

    this.isHandbrakeActive = handbrakeInput;

    // 2. Base axle torque distribution according to brake bias
    const totalBrakeTorque = this.config.maxBrakeTorque * this.pedalPosition;
    const frontAxleTorque = totalBrakeTorque * this.config.frontBias;
    const rearAxleTorque = totalBrakeTorque * (1.0 - this.config.frontBias);

    // Each wheel's nominal hydraulic request
    const nominalTorques: [number, number, number, number] = [
      frontAxleTorque * 0.5, // FL
      frontAxleTorque * 0.5, // FR
      rearAxleTorque * 0.5,  // RL
      rearAxleTorque * 0.5,  // RR
    ];

    // 3. ABS Slip Modulation per wheel
    const absPeriod = 1.0 / this.config.absPulseFrequency;
    this.absGlobalActive = false;

    for (let i = 0; i < 4; i++) {
      const wheel = wheelsDyn[i];
      let pressureScale = 1.0;

      // ABS eligibility: system enabled, car moving above minimum speed, and brake pedal applied
      const canAbsEngage =
        this.config.absEnabled &&
        vehicleSpeedMs > this.config.absMinSpeedMs &&
        this.pedalPosition > 0.08;

      // Check slip ratio (magnitude of slip exceeds threshold)
      const isSlipping = Math.abs(wheel.slipRatio) > this.config.absSlipThreshold;

      if (canAbsEngage && isSlipping) {
        this.absActive[i] = true;
        this.absGlobalActive = true;
        this.absTimers[i] += dt;

        // Rapid pressure modulation cycle:
        // Dump phase (first half of period): drop pressure to 25%
        // Repressurize phase (second half of period): ramp pressure back to 85%
        const phase = (this.absTimers[i] % absPeriod) / absPeriod;
        if (phase < 0.45) {
          pressureScale = 0.22; // Valve dump
        } else {
          pressureScale = lerp(0.35, 0.85, (phase - 0.45) / 0.55); // Pulse recharge
        }
      } else {
        this.absActive[i] = false;
        this.absTimers[i] = 0;
        pressureScale = 1.0;
      }

      this.wheelBrakeTorques[i] = nominalTorques[i] * pressureScale;
    }

    // 4. Rear Handbrake Application
    // Handbrake acts directly via mechanical cable on rear wheels without ABS modulation
    if (this.isHandbrakeActive) {
      this.wheelBrakeTorques[2] = Math.max(this.wheelBrakeTorques[2], this.config.handbrakeTorque);
      this.wheelBrakeTorques[3] = Math.max(this.wheelBrakeTorques[3], this.config.handbrakeTorque);
    }
  }
}
