/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

export interface VehicleAudioConfig {
  sampleUrl?: string;
  idleRpm?: number;
  redlineRpm?: number;
  minPlaybackRate?: number;
  maxPlaybackRate?: number;
  minFilterCutoffHz?: number;
  maxFilterCutoffHz?: number;
  masterVolume?: number;
  idleGainFloor?: number;
}

export const DEFAULT_AUDIO_CONFIG: VehicleAudioConfig = {
  sampleUrl: '',
  idleRpm: 550,
  redlineRpm: 7000,
  minPlaybackRate: 0.0,
  maxPlaybackRate: 0.0,
  minFilterCutoffHz: 0.0,
  maxFilterCutoffHz: 0.0,
  masterVolume: 0.0,
  idleGainFloor: 0.0,
};

/**
 * Pure mathematical helper stubs for backward compatibility.
 * All return zero / silent values.
 */
export function calculatePlaybackRate(): number {
  return 0.0;
}

export function calculateLpfCutoff(): number {
  return 0.0;
}

export function calculateEngineGain(): number {
  return 0.0;
}

export function calculateLowRangeGearMesh(): { active: boolean; centerFreqHz: number; volume: number } {
  return { active: false, centerFreqHz: 0.0, volume: 0.0 };
}

export function calculateShiftClunkParams(): {
  durationSec: number;
  startFreqHz: number;
  endFreqHz: number;
  maxVolume: number;
} {
  return {
    durationSec: 0.0,
    startFreqHz: 0.0,
    endFreqHz: 0.0,
    maxVolume: 0.0,
  };
}

export function createSeamlessLoopBuffer(
  _ctx: AudioContext,
  sourceBuffer: AudioBuffer
): AudioBuffer {
  return sourceBuffer;
}

/**
 * VehicleAudio System — Stage 11 (Surgical Revision: Complete Vehicle Silence).
 *
 * All combustion oscillators, sample buffer loaders, engine filter chains,
 * shift clunk synthesis, and gear mesh noise nodes have been completely stripped.
 * AudioContext is kept clean and idle. All public methods are safe, empty no-ops
 * so that callers from physics loops and control handlers execute without errors.
 */
export class VehicleAudio {
  public readonly config: VehicleAudioConfig;

  // Clean, idle Web Audio context
  private ctx: AudioContext | null = null;
  private isUnlocked: boolean = false;
  private isDisposed: boolean = false;

  constructor(config: Partial<VehicleAudioConfig> = {}) {
    this.config = { ...DEFAULT_AUDIO_CONFIG, ...config };
  }

  /**
   * Initializes AudioContext in a clean, idle state.
   * Connects ZERO oscillators, filters, buffers, or gain nodes to destination.
   */
  public init(): void {
    if (this.ctx) return;

    const AudioContextClass =
      typeof window !== 'undefined'
        ? window.AudioContext ||
          (window as unknown as { webkitAudioContext: typeof AudioContext }).webkitAudioContext
        : null;

    if (!AudioContextClass) return;

    try {
      this.ctx = new AudioContextClass();
    } catch {
      // Idle failsafe
    }
  }

  /**
   * Resumes AudioContext on user interaction while keeping it completely idle.
   */
  public unlock(): void {
    if (this.isDisposed || this.isUnlocked) return;

    if (!this.ctx) {
      this.init();
    }

    if (this.ctx && this.ctx.state === 'suspended') {
      this.ctx.resume().catch(() => {});
    }

    this.isUnlocked = true;
  }

  /**
   * Safe no-op stub: shift clunk synth triggers removed.
   */
  public playShiftClunk(_volume: number = 0.30): void {
    // No-op: complete silence
  }

  /**
   * Alias for playShiftClunk for API compatibility. Safe no-op stub.
   */
  public triggerShiftClunk(_volume: number = 0.30): void {
    // No-op: complete silence
  }

  /**
   * Safe no-op stub: all engine sound synthesis removed.
   * Produces zero audio processing, zero node manipulations, zero allocations.
   */
  public update(
    _engineRpmOrOptions?: number | {
      engineRpm: number;
      throttle?: number;
      dt?: number;
      isShifting?: boolean;
      speedKmh?: number;
      rangeMode?: 'high' | 'low';
    },
    _throttleArg: number = 0,
    _dtArg: number = 0.016,
    _isShiftingArg: boolean = false,
    _speedKmhArg: number = 0,
    _rangeModeArg: 'high' | 'low' = 'high'
  ): void {
    // No-op: complete silence
  }

  /**
   * Clean teardown of audio context.
   */
  public dispose(): void {
    if (this.isDisposed) return;
    this.isDisposed = true;

    try {
      if (this.ctx) {
        this.ctx.close().catch(() => {});
        this.ctx = null;
      }
    } catch {
      // Teardown failsafe
    }
  }

  public getContextState(): string {
    return this.ctx ? this.ctx.state : 'uninitialized';
  }

  public isAudioUnlocked(): boolean {
    return this.isUnlocked;
  }

  public isSampleLoaded(): boolean {
    return false;
  }

  public getPlaybackRate(): number {
    return 0.0;
  }

  public getFilterCutoff(): number {
    return 0.0;
  }
}

// Dual export aliases for seamless import across codebase
export { VehicleAudio as VehicleAudioSystem };
export default VehicleAudio;
