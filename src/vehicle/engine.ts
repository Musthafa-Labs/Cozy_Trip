/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import { clamp, lerp } from '../utils/math';

export interface TorqueCurvePoint {
  rpm: number;
  torque: number; // N*m at wide-open throttle (WOT)
}

export interface EngineConfig {
  idleRpm: number;           // Idle speed in RPM (default: 750)
  stallRpm: number;          // Stall threshold in RPM (default: 450)
  peakTorqueRpm: number;     // RPM of peak torque (default: 2000)
  redlineRpm: number;        // Maximum allowed RPM (default: 4400)
  revLimiterCutRpm: number;  // Hard cut threshold (default: 4400)
  revLimiterResumeRpm: number; // Threshold to resume fuel injection (default: 4250)
  inertia: number;           // Engine flywheel + rotating assembly inertia in kg*m^2 (default: 0.35)
  throttleResponseTau: number; // 1st-order throttle response filter time constant in seconds (default: 0.08)
  engineBrakingBase: number; // Base engine friction braking in N*m (default: 28.0)
  engineBrakingRpmFactor: number; // Speed-dependent engine braking factor N*m/RPM (default: 0.022)
  torqueCurve: TorqueCurvePoint[]; // Piecewise torque curve points
}

// Heavy-duty 4.2L Turbo-Diesel inline-6 expedition engine profile
// Characterized by strong low-end torque, flat peak plateau around 1800-2400 RPM,
// smooth tapering off at higher revs, and governed fuel cut at 4400 RPM.
export const DEFAULT_TORQUE_CURVE: TorqueCurvePoint[] = [
  { rpm: 0, torque: 220 },
  { rpm: 500, torque: 310 },
  { rpm: 750, torque: 360 },   // Healthy idle torque reserve
  { rpm: 1200, torque: 430 },
  { rpm: 1800, torque: 480 },  // Peak torque plateau begins
  { rpm: 2200, torque: 480 },  // Peak torque plateau
  { rpm: 2600, torque: 460 },
  { rpm: 3200, torque: 400 },
  { rpm: 3800, torque: 320 },
  { rpm: 4200, torque: 220 },
  { rpm: 4400, torque: 150 },  // Redline / governor threshold
  { rpm: 4800, torque: 40 },
];

export class Engine {
  public readonly config: EngineConfig;

  // Running & Stall State
  public isRunning: boolean = true;
  public isStalled: boolean = false;

  // Kinematic & Dynamic State
  public rpm: number = 750;
  public omega: number = (750 * 2 * Math.PI) / 60; // rad/s
  public throttle: number = 0;                     // Smoothed throttle [0 .. 1.0]
  public outputTorque: number = 0;                 // Net output torque at flywheel (N*m)
  public rawCombustionTorque: number = 0;          // Gross combustion torque (N*m)
  public engineBrakingTorque: number = 0;          // Retarding torque (N*m)
  public isRevLimiterActive: boolean = false;

  constructor(config: Partial<EngineConfig> = {}) {
    this.config = {
      idleRpm: config.idleRpm ?? 750,
      stallRpm: config.stallRpm ?? 450,
      peakTorqueRpm: config.peakTorqueRpm ?? 2000,
      redlineRpm: config.redlineRpm ?? 4400,
      revLimiterCutRpm: config.revLimiterCutRpm ?? 4400,
      revLimiterResumeRpm: config.revLimiterResumeRpm ?? 4250,
      inertia: config.inertia ?? 0.35,
      throttleResponseTau: config.throttleResponseTau ?? 0.08,
      engineBrakingBase: config.engineBrakingBase ?? 28.0,
      engineBrakingRpmFactor: config.engineBrakingRpmFactor ?? 0.022,
      torqueCurve: config.torqueCurve ?? DEFAULT_TORQUE_CURVE,
    };

    this.rpm = this.config.idleRpm;
    this.omega = (this.rpm * 2 * Math.PI) / 60;
  }

  /**
   * Starts the engine (ignition on, initial idle speed).
   */
  public start(): void {
    this.isRunning = true;
    this.isStalled = false;
    this.rpm = this.config.idleRpm;
    this.omega = (this.rpm * 2 * Math.PI) / 60;
    this.outputTorque = 0;
    this.rawCombustionTorque = 0;
    this.isRevLimiterActive = false;
  }

  /**
   * Stalls or shuts off the engine (combustion terminates, no driving torque produced).
   */
  public stall(): void {
    this.isRunning = false;
    this.isStalled = true;
    this.outputTorque = 0;
    this.rawCombustionTorque = 0;
    this.isRevLimiterActive = false;
  }

  /**
   * Toggles engine ignition state (starts if stopped/stalled; turns off if running).
   */
  public toggleIgnition(): void {
    if (this.isRunning) {
      this.stall();
    } else {
      this.start();
    }
  }

  public reset(): void {
    this.start();
    this.throttle = 0;
    this.engineBrakingTorque = 0;
  }

  /**
   * Interpolates the engine wide-open throttle (WOT) torque from the torque curve table.
   */
  public getWOTTorque(rpm: number): number {
    const curve = this.config.torqueCurve;
    if (rpm <= curve[0].rpm) return curve[0].torque;
    const last = curve[curve.length - 1];
    if (rpm >= last.rpm) return last.torque;

    for (let i = 0; i < curve.length - 1; i++) {
      const p0 = curve[i];
      const p1 = curve[i + 1];
      if (rpm >= p0.rpm && rpm <= p1.rpm) {
        const factor = (rpm - p0.rpm) / (p1.rpm - p0.rpm);
        return lerp(p0.torque, p1.torque, factor);
      }
    }
    return curve[0].torque;
  }

  /**
   * Updates engine RPM and output torque given throttle pedal input and clutch reaction load.
   *
   * @param throttleInput Player throttle pedal position [0 .. 1.0]
   * @param clutchReactionTorque Reaction torque transmitted through clutch (N*m)
   * @param clutchEngagement Clutch lockup factor [0 = disengaged, 1.0 = locked]
   * @param targetDrivetrainRpm Wheel-coupled RPM from driveshaft
   * @param dt Fixed physics timestep in seconds (1/120)
   */
  public update(
    throttleInput: number,
    clutchReactionTorque: number,
    clutchEngagement: number,
    targetDrivetrainRpm: number,
    dt: number
  ): void {
    // 1. Throttle input filtering (smooth 1st-order response)
    const clampedInput = clamp(throttleInput, 0, 1.0);
    const alpha = clamp(dt / (this.config.throttleResponseTau + dt), 0.05, 1.0);
    this.throttle = lerp(this.throttle, clampedInput, alpha);

    // 2. Stalled or Engine Off State:
    // When stalled, combustion ceases entirely and the rotating assembly spins down to zero.
    if (!this.isRunning) {
      this.rawCombustionTorque = 0;
      this.outputTorque = 0;
      this.isRevLimiterActive = false;
      this.engineBrakingTorque = this.config.engineBrakingBase + this.config.engineBrakingRpmFactor * this.rpm;

      if (clutchEngagement >= 0.95) {
        // Locked to driveshaft: RPM follows vehicle motion directly
        this.rpm = clamp(targetDrivetrainRpm, 0, this.config.redlineRpm);
        this.omega = (this.rpm * 2 * Math.PI) / 60;
      } else {
        // Freely spinning down under engine internal friction
        const spinDownOmega = (this.engineBrakingTorque / this.config.inertia) * dt;
        this.omega = Math.max(0, this.omega - spinDownOmega);
        this.rpm = (this.omega * 60) / (2 * Math.PI);
      }
      return;
    }

    // 3. Diesel Governor / Rev Limiter Logic
    if (this.rpm >= this.config.revLimiterCutRpm) {
      this.isRevLimiterActive = true;
    } else if (this.rpm <= this.config.revLimiterResumeRpm) {
      this.isRevLimiterActive = false;
    }

    // 4. Wide-open-throttle torque & net combustion torque
    const wotTorque = this.getWOTTorque(this.rpm);
    let combustionTorque = wotTorque * this.throttle;

    // Fuel cut during governor activation
    if (this.isRevLimiterActive) {
      combustionTorque = 0;
    }

    // 5. Closed-loop Diesel Idle Governor
    // When idling at low throttle, inject governor torque to sustain idle speed (750 RPM).
    // The governor has realistic limited authority (max ~75 N*m), so high driveline load can lug it down.
    if (this.throttle < 0.15 && this.rpm <= this.config.idleRpm + 40) {
      const idleDeficit = Math.max(0, this.config.idleRpm - this.rpm);
      const idleGovernorTorque = clamp(idleDeficit * 1.1, 0, 75.0);
      combustionTorque = Math.max(combustionTorque, idleGovernorTorque);
    }

    this.rawCombustionTorque = combustionTorque;

    // 6. Engine friction & engine braking torque (always opposes rotation)
    this.engineBrakingTorque = this.config.engineBrakingBase + this.config.engineBrakingRpmFactor * this.rpm;

    // 7. Net flywheel torque before clutch
    const netFlywheelTorque = combustionTorque - this.engineBrakingTorque;

    // 8. Dynamic Engine RPM Integration & Realistic Stalling
    if (clutchEngagement >= 0.95) {
      // Locked clutch regime: Engine RPM is rigidly coupled to driveshaft rotation
      const coupledRpm = targetDrivetrainRpm;
      this.rpm = coupledRpm;
      this.omega = (this.rpm * 2 * Math.PI) / 60;
      this.outputTorque = netFlywheelTorque;

      // Stall check under locked clutch: if driveline drags engine below stall threshold
      if (this.rpm < this.config.stallRpm) {
        this.stall();
        return;
      }
    } else {
      // Slipping clutch or neutral/idle regime:
      const slipRpm = this.rpm - targetDrivetrainRpm;
      const normalizedSlip = clamp(slipRpm / 350, -1.0, 1.0);
      const clutchSlipTorque = normalizedSlip * 480.0 * clutchEngagement;

      const netTorqueOnFlywheel = netFlywheelTorque - clutchSlipTorque;
      const dOmega = (netTorqueOnFlywheel / this.config.inertia) * dt;
      this.omega += dOmega;

      this.rpm = (this.omega * 60) / (2 * Math.PI);

      // Stall check under clutch load:
      // If clutch is loaded (engagement > 0.15) and driveshaft is lugging the engine below stall threshold
      if (clutchEngagement > 0.15 && this.rpm < this.config.stallRpm) {
        this.stall();
        return;
      }

      // Idle governor catch when clutch is disengaged or vehicle in neutral
      const minOmega = (this.config.idleRpm * 2 * Math.PI) / 60;
      if (this.omega < minOmega && clutchEngagement <= 0.15) {
        this.omega = minOmega;
        this.rpm = this.config.idleRpm;
      }

      // Cap at governor max limit
      const maxOmega = ((this.config.redlineRpm + 100) * 2 * Math.PI) / 60;
      if (this.omega > maxOmega) {
        this.omega = maxOmega;
        this.rpm = (this.omega * 60) / (2 * Math.PI);
      }

      this.outputTorque = clutchSlipTorque;
    }

    // Safety: at zero throttle and vehicle standstill with clutch disengaged, output torque must not creep
    if (this.throttle < 0.005 && clutchEngagement < 0.1) {
      this.outputTorque = 0;
    }
  }
}
