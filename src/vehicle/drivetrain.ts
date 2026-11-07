/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import { clamp, lerp } from '../utils/math';
import { Engine } from './engine';
import { WheelDynamics } from './wheel';

export type DrivetrainLayout = 'AWD' | 'RWD' | 'FWD';
export type TransmissionMode = 'auto' | 'manual';
export type RangeMode = 'high' | 'low';

export interface TransmissionConfig {
  gearRatios: number[];      // Index: 0 = Neutral, 1 = 1st, ..., 6 = 6th
  reverseRatio: number;      // Reverse gear ratio (negative, e.g. -3.75)
  finalDriveRatio: number;   // Final drive ratio (differential ring & pinion, e.g. 3.91)
  lowRangeRatio: number;     // Low range transfer case multiplier (default: 2.5)
  lowRangeMaxGear: number;   // Highest forward gear selectable in low range (default: 4)
  drivetrainEfficiency: number; // Mechanical efficiency (e.g. 0.92)
  shiftDuration: number;     // Gear change shift time in seconds (e.g. 0.22 s)
  minShiftInterval: number;  // Minimum time between completed automatic shifts in seconds (default: 1.0 s)
  clutchEngageStartRpm: number; // RPM where clutch starts grabbing (e.g. 1050 RPM)
  clutchEngageLockRpm: number;  // RPM where clutch achieves full lockup (e.g. 2100 RPM)
  clutchMaxTorqueCapacity: number; // Max clutch friction torque in N*m (e.g. 600 N*m)
  differentialType: 'open' | 'lsd'; // Differential type
  lsdStiffness: number;      // Limited Slip torque transfer factor N*m/(rad/s) (e.g. 45.0)
  lsdMaxBiasRatio: number;   // Maximum LSD torque split ratio (e.g. 3.0)
}

export const DEFAULT_TRANSMISSION_CONFIG: TransmissionConfig = {
  // Index 0 is Neutral, 1..6 are forward gears
  gearRatios: [0, 3.98, 2.61, 1.82, 1.37, 1.00, 0.81],
  reverseRatio: -3.75,
  finalDriveRatio: 3.91,
  lowRangeRatio: 2.5,
  lowRangeMaxGear: 4,
  drivetrainEfficiency: 0.92,
  shiftDuration: 0.22,
  minShiftInterval: 1.0,
  clutchEngageStartRpm: 850,
  clutchEngageLockRpm: 1600,
  clutchMaxTorqueCapacity: 650,
  differentialType: 'lsd',
  lsdStiffness: 50.0,
  lsdMaxBiasRatio: 2.8,
};

export class Drivetrain {
  public readonly config: TransmissionConfig;

  // Configuration
  public layout: DrivetrainLayout = 'AWD';
  public mode: TransmissionMode = 'auto';
  public rangeMode: RangeMode = 'high';
  public minShiftInterval: number = 1.0;
  public timeSinceShift: number = 10.0; // Initialized so first shift has no delay

  // Gearbox State
  // -1 = Reverse, 0 = Neutral, 1..6 = Forward
  public currentGear: number = 1;
  public targetGear: number = 1;
  public isShifting: boolean = false;
  public shiftProgress: number = 0; // 0 to 1.0

  // Clutch State
  public clutchEngagement: number = 0; // 0 = disengaged, 1.0 = locked
  public clutchReactionTorque: number = 0; // N*m transmitted across clutch
  public targetDrivetrainRpm: number = 900; // Calculated from wheel angular velocities

  // Torque distribution to 4 wheels (FL, FR, RL, RR) in N*m
  public wheelTorques: [number, number, number, number] = [0, 0, 0, 0];
  public driveshaftTorque: number = 0;

  constructor(config: Partial<TransmissionConfig> = {}) {
    this.config = { ...DEFAULT_TRANSMISSION_CONFIG, ...config };
    this.minShiftInterval = this.config.minShiftInterval ?? 1.0;
    this.timeSinceShift = this.minShiftInterval;
  }

  public reset(): void {
    this.rangeMode = 'high';
    this.currentGear = 1;
    this.targetGear = 1;
    this.isShifting = false;
    this.shiftProgress = 0;
    this.timeSinceShift = this.minShiftInterval;
    this.clutchEngagement = 0;
    this.clutchReactionTorque = 0;
    this.targetDrivetrainRpm = 900;
    this.wheelTorques = [0, 0, 0, 0];
    this.driveshaftTorque = 0;
  }

  /**
   * Single source-of-truth function for overall transmission ratio:
   * gearRatio * finalDriveRatio * (rangeMode === 'low' ? lowRangeRatio : 1)
   * Handles forward gears, reverse, driveshaft torque, target RPM, and clutch calculations.
   */
  public getOverallRatio(gear: number = this.currentGear): number {
    if (gear === 0) return 0;
    const baseGearRatio = gear === -1 ? this.config.reverseRatio : (this.config.gearRatios[gear] ?? 1.0);
    const rangeMultiplier = this.rangeMode === 'low' ? this.config.lowRangeRatio : 1.0;
    return baseGearRatio * this.config.finalDriveRatio * rangeMultiplier;
  }

  /**
   * Toggles between HIGH and LOW transfer case range.
   * Switching is permitted only when absolute longitudinal speed is < 1.0 m/s.
   * If transitioning into LOW while in a gear above lowRangeMaxGear, clamps to lowRangeMaxGear.
   */
  public toggleRange(forwardSpeedMs: number): boolean {
    if (Math.abs(forwardSpeedMs) >= 1.0) {
      return false;
    }
    this.rangeMode = this.rangeMode === 'high' ? 'low' : 'high';
    if (this.rangeMode === 'low' && this.currentGear > this.config.lowRangeMaxGear) {
      this.currentGear = this.config.lowRangeMaxGear;
      this.targetGear = this.config.lowRangeMaxGear;
      this.isShifting = false;
      this.shiftProgress = 0;
    }
    return true;
  }

  /**
   * Request shifting to a specific gear with safety checks and low-range limit enforcement.
   */
  public requestGear(gear: number, forwardSpeedMs: number, engineRpm: number): boolean {
    if (gear < -1 || gear > 6) return false;
    if (this.rangeMode === 'low' && gear > this.config.lowRangeMaxGear) return false;
    if (gear === this.currentGear) return true;

    // Safety: prevent selecting reverse if moving forward at > 2.5 km/h (0.7 m/s)
    if (gear === -1 && forwardSpeedMs > 0.7) {
      return false;
    }

    // Safety: prevent shifting into forward gear if moving backward fast
    if (gear > 0 && forwardSpeedMs < -1.5) {
      return false;
    }

    // Safety: prevent downshift over-rev (where new gear would push RPM > 4300)
    if (gear > 0 && gear < this.currentGear) {
      const currentOverall = this.getOverallRatio(this.currentGear);
      const newOverall = this.getOverallRatio(gear);
      if (Math.abs(currentOverall) > 0.001) {
        const projectedRpm = (engineRpm * newOverall) / currentOverall;
        if (projectedRpm > 4300) {
          return false;
        }
      }
    }

    this.targetGear = gear;
    this.isShifting = true;
    this.shiftProgress = 0;
    return true;
  }

  public shiftUp(forwardSpeedMs: number, engineRpm: number): boolean {
    if (this.currentGear === -1) return this.requestGear(1, forwardSpeedMs, engineRpm);
    const maxGear = this.rangeMode === 'low' ? this.config.lowRangeMaxGear : 6;
    if (this.currentGear >= maxGear) return false;
    return this.requestGear(this.currentGear + 1, forwardSpeedMs, engineRpm);
  }

  public shiftDown(forwardSpeedMs: number, engineRpm: number): boolean {
    if (this.currentGear === 1) return this.requestGear(-1, forwardSpeedMs, engineRpm);
    if (this.currentGear <= -1) return false;
    return this.requestGear(this.currentGear - 1, forwardSpeedMs, engineRpm);
  }

  /**
   * Evaluates automatic transmission shifting based on throttle, engine RPM, and predicted-RPM hysteresis.
   */
  private updateAutomaticShifting(
    engineRpm: number,
    throttle: number,
    forwardSpeedMs: number
  ): void {
    if (this.mode !== 'auto' || this.isShifting) return;

    // Minimum time between automatic shifts must have elapsed
    if (this.timeSinceShift < this.minShiftInterval) return;

    // In reverse gear
    if (this.currentGear === -1) {
      // If stopped or rolling forward, keep reverse until player shifts or brakes
      return;
    }

    if (this.currentGear >= 1) {
      // Diesel-adapted shift points: cruise at 1850-2400 RPM, peak power shift at 3800 RPM
      const upshiftThreshold = lerp(1850, 3800, throttle);
      const downshiftThreshold = lerp(1100, 2200, throttle);

      const maxAllowedGear = this.rangeMode === 'low' ? this.config.lowRangeMaxGear : 6;
      const currentRatio = this.getOverallRatio(this.currentGear);
      if (Math.abs(currentRatio) < 0.001) return;

      // Upshift condition: single gear, next forward gear exists within range limits
      if (this.currentGear < maxAllowedGear && engineRpm > upshiftThreshold) {
        const nextGear = this.currentGear + 1;
        const nextRatio = this.getOverallRatio(nextGear);
        if (Math.abs(nextRatio) > 0.001) {
          const predictedRpm = (engineRpm * nextRatio) / currentRatio;
          if (predictedRpm >= downshiftThreshold + 300) {
            this.requestGear(nextGear, forwardSpeedMs, engineRpm);
            return;
          }
        }
      }

      // Downshift condition: single gear, lower forward gear exists
      if (this.currentGear > 1 && engineRpm < downshiftThreshold) {
        const lowerGear = this.currentGear - 1;
        const lowerRatio = this.getOverallRatio(lowerGear);
        if (Math.abs(lowerRatio) > 0.001) {
          const predictedRpm = (engineRpm * lowerRatio) / currentRatio;
          if (predictedRpm <= upshiftThreshold - 300 && predictedRpm < 4200) {
            this.requestGear(lowerGear, forwardSpeedMs, engineRpm);
            return;
          }
        }
      }
    }
  }

  /**
   * Updates transmission gear transitions, automatic clutch, and differential torque split.
   */
  public update(
    engine: Engine,
    wheelsDyn: WheelDynamics[],
    forwardSpeedMs: number,
    dt: number,
    advanceState: boolean = true
  ): void {
    // 1. Shift duration progression and automatic shifting state advancement
    if (advanceState) {
      this.timeSinceShift += dt;

      if (this.isShifting) {
        this.shiftProgress += dt / this.config.shiftDuration;
        if (this.shiftProgress >= 1.0) {
          this.currentGear = this.targetGear;
          this.isShifting = false;
          this.shiftProgress = 0;
          this.timeSinceShift = 0;
        }
      } else {
        this.updateAutomaticShifting(engine.rpm, engine.throttle, forwardSpeedMs);
      }
    }

    // 2. Wheel angular velocity feedback from driven axle(s)
    let drivenOmegaSum = 0;
    let drivenWheelCount = 0;

    // Check front wheels
    if (this.layout === 'AWD' || this.layout === 'FWD') {
      drivenOmegaSum += wheelsDyn[0].omega + wheelsDyn[1].omega;
      drivenWheelCount += 2;
    }
    // Check rear wheels
    if (this.layout === 'AWD' || this.layout === 'RWD') {
      drivenOmegaSum += wheelsDyn[2].omega + wheelsDyn[3].omega;
      drivenWheelCount += 2;
    }

    const avgDrivenOmega = drivenWheelCount > 0 ? drivenOmegaSum / drivenWheelCount : 0;
    const overallRatio = this.getOverallRatio();

    // Projected engine RPM if clutch was fully locked
    if (Math.abs(overallRatio) > 0.001) {
      const driveshaftOmega = avgDrivenOmega * Math.abs(overallRatio);
      this.targetDrivetrainRpm = (driveshaftOmega * 60) / (2 * Math.PI);
    } else {
      this.targetDrivetrainRpm = engine.config.idleRpm;
    }

    // 3. Clutch Modulation
    // Clutch engagement depends on RPM, shift state, transmission mode, and vehicle speed
    if (this.currentGear === 0 || this.isShifting) {
      // Neutral or mid-shift: clutch is released / slipping
      this.clutchEngagement = this.isShifting ? 0.15 : 0.0;
    } else if (this.mode === 'auto') {
      // Automatic Mode: fluid coupling / auto-clutch
      // Progressive engagement between start RPM (850) and lockup RPM (1600)
      const lockupRange = this.config.clutchEngageLockRpm - this.config.clutchEngageStartRpm;
      const rawEngagement = (engine.rpm - this.config.clutchEngageStartRpm) / lockupRange;
      this.clutchEngagement = clamp(rawEngagement, 0, 1.0);

      // If vehicle is rolling at speed in gear, clutch stays locked
      if (Math.abs(forwardSpeedMs) > 1.8 && engine.rpm > engine.config.idleRpm + 100) {
        this.clutchEngagement = Math.max(this.clutchEngagement, 0.95);
      }
    } else {
      // Manual Mode:
      // In manual mode, the clutch is mechanically coupled without automatic disengagement.
      if (Math.abs(forwardSpeedMs) > 1.5) {
        // While moving in gear, clutch remains firmly locked
        this.clutchEngagement = 1.0;
      } else {
        // Near standstill in manual gear:
        const isLowLaunchGear = this.currentGear === 1 || this.currentGear === -1 || (this.rangeMode === 'low' && this.currentGear <= 2);
        if (isLowLaunchGear) {
          const lockupRange = this.config.clutchEngageLockRpm - this.config.clutchEngageStartRpm;
          const rawEngagement = (engine.rpm - this.config.clutchEngageStartRpm) / lockupRange;
          // In 1st gear manual: clutch bites at minimum 0.25. If brakes held or stationary without throttle,
          // targetDrivetrainRpm is 0, so driveline drag pulls engine below 450 RPM and stalls!
          this.clutchEngagement = clamp(rawEngagement, 0.25, 1.0);
        } else {
          // Attempting to move or stopped in 2nd, 3rd, 4th, etc. in manual mode:
          // Heavy clutch load (0.80) lugs the engine immediately below stall threshold
          this.clutchEngagement = 0.80;
        }
      }
    }

    // 4. Clutch Torque Transfer
    // Torque capacity transmitted across clutch
    const maxClutchT = this.config.clutchMaxTorqueCapacity * this.clutchEngagement;
    this.clutchReactionTorque = clamp(engine.rawCombustionTorque, -maxClutchT, maxClutchT);

    // Transmitted torque through clutch into gearbox
    let transmittedTorque = engine.outputTorque;

    // Shift torque reduction (smooth dip to prevent violent acceleration spikes)
    if (this.isShifting) {
      transmittedTorque *= 0.15;
    }

    // 5. Driveshaft Torque Multiplication
    // T_driveshaft = T_engine * gearRatio * finalDrive * rangeMultiplier * efficiency
    this.driveshaftTorque = transmittedTorque * overallRatio * this.config.drivetrainEfficiency;

    // Safety: no driveshaft torque in neutral or at zero throttle standstill
    if (this.currentGear === 0 || (engine.throttle < 0.005 && Math.abs(forwardSpeedMs) < 0.05)) {
      this.driveshaftTorque = 0;
    }

    // 6. Center Differential Split (AWD / RWD / FWD)
    let frontAxleTorque = 0;
    let rearAxleTorque = 0;

    switch (this.layout) {
      case 'AWD':
        frontAxleTorque = this.driveshaftTorque * 0.45;
        rearAxleTorque = this.driveshaftTorque * 0.55;
        break;
      case 'RWD':
        frontAxleTorque = 0;
        rearAxleTorque = this.driveshaftTorque;
        break;
      case 'FWD':
        frontAxleTorque = this.driveshaftTorque;
        rearAxleTorque = 0;
        break;
    }

    // 7. Axle Differentials (Open vs Limited-Slip Differential)
    // Front Axle: FL (index 0) and FR (index 1)
    const [tFL, tFR] = this.splitAxleTorque(
      frontAxleTorque,
      wheelsDyn[0].omega,
      wheelsDyn[1].omega
    );

    // Rear Axle: RL (index 2) and RR (index 3)
    const [tRL, tRR] = this.splitAxleTorque(
      rearAxleTorque,
      wheelsDyn[2].omega,
      wheelsDyn[3].omega
    );

    this.wheelTorques[0] = tFL;
    this.wheelTorques[1] = tFR;
    this.wheelTorques[2] = tRL;
    this.wheelTorques[3] = tRR;
  }

  /**
   * Splits torque between left and right wheels of an axle with Open or LSD behavior.
   */
  private splitAxleTorque(
    axleTorque: number,
    omegaLeft: number,
    omegaRight: number
  ): [number, number] {
    if (Math.abs(axleTorque) < 0.01) {
      return [0, 0];
    }

    const halfTorque = axleTorque * 0.5;

    if (this.config.differentialType === 'open') {
      // Standard open differential splits torque 50/50
      return [halfTorque, halfTorque];
    }

    // Limited Slip Differential (LSD):
    // Transfers torque from the faster wheel (slipping) to the slower wheel (gripping)
    const deltaOmega = omegaLeft - omegaRight; // rad/s
    const maxTransfer = Math.abs(axleTorque) * (this.config.lsdMaxBiasRatio / (1 + this.config.lsdMaxBiasRatio));
    const transferTorque = clamp(deltaOmega * this.config.lsdStiffness, -maxTransfer, maxTransfer);

    const tLeft = halfTorque - transferTorque;
    const tRight = halfTorque + transferTorque;

    return [tLeft, tRight];
  }
}
