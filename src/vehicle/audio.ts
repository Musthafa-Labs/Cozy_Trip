/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import { clamp, lerp } from '../utils/math';

export interface VehicleAudioConfig {
  sampleUrl: string;         // Asset path to truck audio file
  idleRpm: number;           // Baseline idle RPM (default: 550)
  redlineRpm: number;        // Maximum redline RPM (default: 7000)
  minPlaybackRate: number;   // Playback rate at idle (default: 1.0)
  maxPlaybackRate: number;   // Playback rate at redline (default: 2.2)
  minFilterCutoffHz: number; // Low-pass filter cutoff at idle (default: 180.0 Hz)
  maxFilterCutoffHz: number; // Low-pass filter cutoff at high speed (default: 450.0 Hz)
  masterVolume: number;      // Master headroom ceiling (default: 0.38 max output)
  idleGainFloor: number;     // Baseline idle gain (default: 0.28)
}

export const DEFAULT_AUDIO_CONFIG: VehicleAudioConfig = {
  sampleUrl: '/assets/aistudio/mixkit-truck-accelerating-1617_ef9f8673.wav',
  idleRpm: 550,
  redlineRpm: 7000,
  minPlaybackRate: 1.0,
  maxPlaybackRate: 2.2,
  minFilterCutoffHz: 180.0,
  maxFilterCutoffHz: 450.0,
  masterVolume: 0.38,
  idleGainFloor: 0.28,
};

/**
 * Pure mathematical helper for dynamic RPM playback rate modulation & gear shift state machine:
 * - Maps vehicle RPM (550 idle to 7000 redline) to playback rate (1.0x to 2.2x).
 * - During gear shifts (isShifting === true), ramps playbackRate down by 22% (0.78 ratio)
 *   over 0.18s to replicate clutch disengagement physics.
 * - Reverse gear: maintains steady baseline rumble (~1.05x–1.2x) regardless of negative speed sign.
 * - LOW Range: compresses speed-to-RPM pitch sweep (shorter gear acceleration bands).
 */
export function calculatePlaybackRate(
  engineRpm: number,
  isShifting: boolean,
  rangeMode: 'high' | 'low' = 'high',
  throttle: number = 0,
  speedKmh: number = 0,
  idleRpm: number = 550,
  redlineRpm: number = 7000,
  minRate: number = 1.0,
  maxRate: number = 2.2
): number {
  const safeRpm = Math.max(engineRpm, 100);
  const rpmFrac = clamp((safeRpm - idleRpm) / (redlineRpm - idleRpm), 0, 1.15);

  let baseRate: number;

  // Reverse gear behavior: steady baseline engine rumble ~1.05x-1.2x regardless of velocity sign
  if (throttle < -0.05) {
    baseRate = clamp(1.05 + Math.abs(throttle) * 0.12, 1.05, 1.20);
  } else if (rangeMode === 'low') {
    // LOW Range: compress vehicle speed-to-RPM pitch sweep (shorter gear acceleration bands)
    const compressedFrac = clamp(rpmFrac * 1.35, 0, 1.15);
    baseRate = lerp(1.02, maxRate * 1.05, compressedFrac);
  } else {
    // HIGH Range: standard highway sweeps
    baseRate = lerp(minRate, maxRate, rpmFrac);
  }

  // Gear Shift: 22% dip during clutch disengagement
  if (isShifting) {
    baseRate *= 0.78;
  }

  return clamp(baseRate, 0.4, 3.5);
}

/**
 * Pure mathematical helper for dynamic Low-Pass Filter cutoff modulation:
 * - 180 Hz at idle to 450 Hz at high speed/load for heavy cast-iron diesel block acoustics.
 * - Masks static sample loop repetition by suppressing mid/high vocal frequencies.
 */
export function calculateLpfCutoff(
  engineRpm: number,
  throttle: number,
  speedKmh: number,
  rangeMode: 'high' | 'low' = 'high',
  minCutoff: number = 180.0,
  maxCutoff: number = 450.0
): number {
  const safeRpm = Math.max(engineRpm, 100);
  const rpmFrac = clamp((safeRpm - 550) / (7000 - 550), 0, 1.0);
  const speedFrac = clamp(Math.abs(speedKmh) / 80.0, 0, 1.0);
  const throttleFrac = Math.abs(throttle);

  const loadFrac = clamp(rpmFrac * 0.45 + speedFrac * 0.35 + throttleFrac * 0.20, 0, 1.0);
  return clamp(lerp(minCutoff, maxCutoff, loadFrac), minCutoff, maxCutoff);
}

/**
 * Pure mathematical helper for master engine gain with strict ceiling and floor:
 * - Baseline idle gain of 0.28 (stationary / reverse / crawling) so low speeds remain clearly audible.
 * - Capped at 0.38 maximum output to prevent distortion and speaker blowouts.
 * - During gear shifts (isShifting === true): ducks by 15% (gain *= 0.85) to eliminate pops or harsh steps.
 */
export function calculateEngineGain(
  throttle: number,
  rpmFrac: number,
  isShifting: boolean = false,
  minFloor: number = 0.28,
  maxCeiling: number = 0.38
): number {
  const throttleSwell = Math.abs(throttle) * 0.08;
  const rpmSwell = clamp(rpmFrac, 0, 1) * 0.02;
  let gain = clamp(minFloor + throttleSwell + rpmSwell, minFloor, maxCeiling);

  // During gear shift disengagement: duck by 15% (gain *= 0.85) to eliminate pops and pitch steps
  if (isShifting) {
    gain *= 0.85;
  }

  return clamp(gain, 0.20, maxCeiling);
}

/**
 * Pure mathematical helper for LOW range gear-mesh whine (bandpass-filtered noise, 120 Hz to 220 Hz):
 * Active strictly in LOW range (L key); silent when stationary or in HIGH range.
 * Zero sine/high-frequency laser tones.
 */
export function calculateLowRangeGearMesh(
  speedKmh: number,
  rangeMode: 'high' | 'low'
): { active: boolean; centerFreqHz: number; volume: number } {
  if (rangeMode !== 'low' || Math.abs(speedKmh) <= 0.3) {
    return { active: false, centerFreqHz: 120.0, volume: 0.0 };
  }
  const speedFrac = clamp(Math.abs(speedKmh) / 45.0, 0, 1.0);
  const centerFreqHz = lerp(120.0, 220.0, speedFrac);
  const volume = clamp(0.015 + 0.030 * speedFrac, 0.0, 0.045);
  return { active: true, centerFreqHz, volume };
}

/**
 * Shift mechanical clunk parameters:
 * 55 Hz -> 15 Hz low-frequency metallic thud (0.07s burst, max volume 0.30).
 */
export function calculateShiftClunkParams(): {
  durationSec: number;
  startFreqHz: number;
  endFreqHz: number;
  maxVolume: number;
} {
  return {
    durationSec: 0.07,
    startFreqHz: 55.0,
    endFreqHz: 15.0,
    maxVolume: 0.30,
  };
}

/**
 * Crossfades an AudioBuffer's end into its start to guarantee click-free, pop-free looping.
 */
export function createSeamlessLoopBuffer(
  ctx: AudioContext,
  sourceBuffer: AudioBuffer,
  crossfadeDurationSec: number = 0.15
): AudioBuffer {
  const sampleRate = sourceBuffer.sampleRate;
  const crossfadeSamples = Math.min(
    Math.floor(sampleRate * crossfadeDurationSec),
    Math.floor(sourceBuffer.length * 0.25)
  );

  const numChannels = sourceBuffer.numberOfChannels;
  const targetLength = sourceBuffer.length - crossfadeSamples;
  if (targetLength <= crossfadeSamples) {
    return sourceBuffer;
  }

  const loopBuffer = ctx.createBuffer(numChannels, targetLength, sampleRate);

  for (let ch = 0; ch < numChannels; ch++) {
    const srcData = sourceBuffer.getChannelData(ch);
    const dstData = loopBuffer.getChannelData(ch);

    // Copy body
    dstData.set(srcData.subarray(0, targetLength));

    // Equal-power crossfade between the very end of srcData and the loop start
    const tailStart = sourceBuffer.length - crossfadeSamples;
    for (let i = 0; i < crossfadeSamples; i++) {
      const frac = i / crossfadeSamples;
      const gainHead = Math.sin(0.5 * Math.PI * frac);
      const gainTail = Math.cos(0.5 * Math.PI * frac);

      const headSample = srcData[i];
      const tailSample = srcData[tailStart + i];

      dstData[i] = headSample * gainHead + tailSample * gainTail;
    }
  }

  return loopBuffer;
}

/**
 * Sample-Based Web Audio Engine for Real Truck Acoustics.
 *
 * Implements Stage 11 (Revision 4 — Sample Audio State Machine & Dropout Fix):
 * 1. Single Persistent Audio Buffer Lifecycle:
 *    Keeps one persistent, continuously looping AudioBufferSourceNode active on AudioContext.currentTime.
 *    NEVER calls .stop() or .disconnect() or recreates the source on shifts, ensuring zero dropouts.
 * 2. Master Volume Attenuation & Gain Ceiling:
 *    Total master gain capped at 0.38 (38% max output) with baseline idle gain of 0.28 (stationary/reverse/crawl).
 * 3. Micro-Jitter & Dynamic LPF Modulation:
 *    0.5% pitch micro-wobble at 3.0 Hz and dynamic LPF (180 Hz idle to 450 Hz high speed) to mask sample loop repetition.
 * 4. Gear Shift State Machine (isShifting === true):
 *    Smoothly ramps playbackRate down by 22% over 0.18s using setTargetAtTime(targetRate, now, 0.04) and ducks
 *    gain by 15% to eliminate pops or harsh pitch steps.
 * 5. Shift Mechanical Clunk:
 *    Synthesized 55 Hz -> 15 Hz low-frequency metallic thud (0.07s burst, max volume 0.30) cleanly at gear engagement.
 * 6. LOW Range Spur-Gear Friction:
 *    Bandpass-filtered noise node (120 Hz to 220 Hz, Q=4.0) tracking vehicle speed (zero laser/beam tones).
 */
export class VehicleAudio {
  public readonly config: VehicleAudioConfig;

  // Web Audio Context & Busses
  private ctx: AudioContext | null = null;
  private masterGain: GainNode | null = null;
  private engineBus: GainNode | null = null;
  private sfxBus: GainNode | null = null;

  // Sample-based Engine Audio Nodes (Persistent lifecycle)
  private engineFilter: BiquadFilterNode | null = null;
  private engineSource: AudioBufferSourceNode | null = null;
  private truckBuffer: AudioBuffer | null = null;
  private isBufferLoaded: boolean = false;
  private isSourceStarted: boolean = false;

  // Shift Clunk & FX
  private shiftClunkBuffer: AudioBuffer | null = null;

  // LOW Range Gear Mesh Friction (Noise + Bandpass)
  private gearMeshNoiseSource: AudioBufferSourceNode | null = null;
  private gearMeshBpf: BiquadFilterNode | null = null;
  private gearMeshGain: GainNode | null = null;

  // Telemetry & Driver State
  private currentRpm: number = 550;
  private currentThrottle: number = 0;
  private currentSpeedKmh: number = 0;
  private currentRangeMode: 'high' | 'low' = 'high';
  private currentIsShifting: boolean = false;
  private currentPlaybackRate: number = 1.0;
  private currentFilterCutoff: number = 180.0;
  private masterPeakGain: number = 0;

  // Lifecycle
  private isUnlocked: boolean = false;
  private isDisposed: boolean = false;

  constructor(config: Partial<VehicleAudioConfig> = {}) {
    this.config = { ...DEFAULT_AUDIO_CONFIG, ...config };
  }

  /**
   * Initializes audio context and begins async decoding of the truck engine audio sample.
   */
  public init(): void {
    if (this.ctx) return;

    const AudioContextClass =
      window.AudioContext ||
      (window as unknown as { webkitAudioContext: typeof AudioContext }).webkitAudioContext;

    if (!AudioContextClass) {
      console.warn('VehicleAudio: Web Audio API not supported in this environment.');
      return;
    }

    try {
      this.ctx = new AudioContextClass();
      this.buildAudioGraph();
      this.loadTruckSample();
    } catch (e) {
      console.warn('VehicleAudio: Failed to initialize AudioContext', e);
    }
  }

  /**
   * Constructs the Web Audio node graph.
   */
  private buildAudioGraph(): void {
    if (!this.ctx) return;
    const now = this.ctx.currentTime;

    // Master Gain Ceiling (capped at 0.38 to prevent distortion)
    this.masterGain = this.ctx.createGain();
    this.masterGain.gain.setValueAtTime(this.config.masterVolume, now);
    this.masterGain.connect(this.ctx.destination);

    // Engine Bus (baseline 0.28 up to 0.38)
    this.engineBus = this.ctx.createGain();
    this.engineBus.gain.setValueAtTime(this.config.idleGainFloor, now);
    this.engineBus.connect(this.masterGain);

    // Dynamic Engine LPF Filter (180 Hz to 450 Hz)
    this.engineFilter = this.ctx.createBiquadFilter();
    this.engineFilter.type = 'lowpass';
    this.engineFilter.Q.setValueAtTime(1.1, now); // Warm low-end resonance
    this.engineFilter.frequency.setValueAtTime(this.config.minFilterCutoffHz, now);
    this.engineFilter.connect(this.engineBus);

    // SFX Bus (Shift clunks)
    this.sfxBus = this.ctx.createGain();
    this.sfxBus.gain.setValueAtTime(0.30, now);
    this.sfxBus.connect(this.masterGain);

    // Shift clunk pre-rendered buffer (55 Hz -> 15 Hz, 0.07s)
    this.buildShiftClunkBuffer();

    // LOW range gear mesh noise & filter
    this.buildGearMeshNoiseSource();
  }

  /**
   * Loads the truck engine WAV sample from the server and performs equal-power loop crossfading.
   */
  private async loadTruckSample(): Promise<void> {
    if (!this.ctx) return;

    try {
      const response = await fetch(this.config.sampleUrl);
      if (!response.ok) {
        throw new Error(`HTTP ${response.status} loading ${this.config.sampleUrl}`);
      }
      const arrayBuf = await response.arrayBuffer();
      const rawDecoded = await this.ctx.decodeAudioData(arrayBuf);

      // Create seamless loop crossfade buffer
      this.truckBuffer = createSeamlessLoopBuffer(this.ctx, rawDecoded, 0.15);
      this.isBufferLoaded = true;

      // If user has already unlocked audio, start persistent playback immediately
      if (this.isUnlocked && !this.isSourceStarted) {
        this.startEngineSource();
      }
    } catch (err) {
      console.warn('VehicleAudio: Failed to load truck sample, falling back to procedural engine buffer', err);
      this.createFallbackEngineBuffer();
      this.isBufferLoaded = true;
      if (this.isUnlocked && !this.isSourceStarted) {
        this.startEngineSource();
      }
    }
  }

  /**
   * Generates a rich fallback diesel engine loop buffer if asset fetch is unavailable.
   */
  private createFallbackEngineBuffer(): void {
    if (!this.ctx) return;
    const sampleRate = this.ctx.sampleRate;
    const duration = 2.0; // 2 seconds seamless loop
    const frameCount = Math.floor(sampleRate * duration);
    const buffer = this.ctx.createBuffer(2, frameCount, sampleRate);

    for (let ch = 0; ch < 2; ch++) {
      const data = buffer.getChannelData(ch);
      let noiseFilter = 0;
      for (let i = 0; i < frameCount; i++) {
        const t = i / sampleRate;
        // Deep 35 Hz cylinder thump + harmonics
        const thud1 = Math.sin(2.0 * Math.PI * 35.0 * t);
        const thud2 = 0.5 * Math.sin(2.0 * Math.PI * 70.0 * t);
        const thud3 = 0.25 * Math.sin(2.0 * Math.PI * 105.0 * t);
        const rawNoise = (Math.random() * 2.0 - 1.0);
        noiseFilter += 0.05 * (rawNoise - noiseFilter);
        data[i] = (thud1 + thud2 + thud3 + noiseFilter * 0.4) * 0.45;
      }
    }

    this.truckBuffer = createSeamlessLoopBuffer(this.ctx, buffer, 0.10);
  }

  /**
   * Starts the SINGLE PERSISTENT looping AudioBufferSourceNode connected into the dynamic engineFilter.
   * NEVER stopped, disconnected, or recreated during vehicle operation.
   */
  private startEngineSource(): void {
    if (!this.ctx || !this.truckBuffer || !this.engineFilter || this.isSourceStarted || this.isDisposed) {
      return;
    }

    try {
      const source = this.ctx.createBufferSource();
      source.buffer = this.truckBuffer;
      source.loop = true;
      source.playbackRate.setValueAtTime(this.currentPlaybackRate, this.ctx.currentTime);
      source.connect(this.engineFilter);
      source.start();

      this.engineSource = source;
      this.isSourceStarted = true;
    } catch {
      // Source start failsafe
    }
  }

  /**
   * Generates a 0.07s (70 ms) heavy metal impact transient buffer (55 Hz -> 15 Hz downward sweep).
   */
  private buildShiftClunkBuffer(): void {
    if (!this.ctx) return;
    const sampleRate = this.ctx.sampleRate;
    const duration = 0.07; // 70 ms
    const frameCount = Math.floor(sampleRate * duration);
    const buffer = this.ctx.createBuffer(1, frameCount, sampleRate);
    const data = buffer.getChannelData(0);

    for (let i = 0; i < frameCount; i++) {
      const t = i / sampleRate;
      const progress = i / frameCount;
      const freq = lerp(55.0, 15.0, progress); // 55 Hz -> 15 Hz
      // Low-frequency triangle transient
      const phase = (freq * t) % 1.0;
      const triangle = 2.0 * Math.abs(2.0 * (phase - Math.floor(phase + 0.5))) - 1.0;
      const env = Math.exp(-t / 0.015);
      data[i] = triangle * env;
    }

    this.shiftClunkBuffer = buffer;
  }

  /**
   * Builds the continuous white noise source for LOW range gear mesh friction.
   * Feeds into a narrow Band-Pass Filter (120 Hz to 220 Hz, Q=4.0).
   */
  private buildGearMeshNoiseSource(): void {
    if (!this.ctx || !this.engineBus) return;
    const sampleRate = this.ctx.sampleRate;
    const duration = 1.0; // 1 second looping noise buffer
    const frameCount = Math.floor(sampleRate * duration);
    const buffer = this.ctx.createBuffer(1, frameCount, sampleRate);
    const data = buffer.getChannelData(0);

    for (let i = 0; i < frameCount; i++) {
      data[i] = Math.random() * 2.0 - 1.0;
    }

    const noiseSource = this.ctx.createBufferSource();
    noiseSource.buffer = buffer;
    noiseSource.loop = true;

    // Narrow Band-Pass Filter (center frequency 120 Hz to 220 Hz, Q = 4.0)
    const bpf = this.ctx.createBiquadFilter();
    bpf.type = 'bandpass';
    bpf.Q.setValueAtTime(4.0, this.ctx.currentTime);
    bpf.frequency.setValueAtTime(120.0, this.ctx.currentTime);

    const gain = this.ctx.createGain();
    gain.gain.setValueAtTime(0.0, this.ctx.currentTime);

    noiseSource.connect(bpf);
    bpf.connect(gain);
    gain.connect(this.engineBus);

    this.gearMeshNoiseSource = noiseSource;
    this.gearMeshBpf = bpf;
    this.gearMeshGain = gain;
  }

  /**
   * Resumes or instantiates AudioContext on first user interaction.
   */
  public unlock(): void {
    if (this.isDisposed || this.isUnlocked) return;

    if (!this.ctx) {
      this.init();
    }

    if (this.ctx && this.ctx.state === 'suspended') {
      this.ctx.resume().catch((err) => {
        console.warn('VehicleAudio: audio resume failed', err);
      });
    }

    if (this.gearMeshNoiseSource) {
      try {
        this.gearMeshNoiseSource.start();
      } catch {
        // Ignored if already started
      }
    }

    this.isUnlocked = true;

    // Start single persistent engine sample loop once unlocked and buffer loaded
    if (this.isBufferLoaded && !this.isSourceStarted) {
      this.startEngineSource();
    }
  }

  /**
   * Triggers a heavy 0.07s metal impact clunk on gear shifts (55 Hz -> 15 Hz, max volume 0.30).
   */
  public playShiftClunk(volume: number = 0.30): void {
    if (!this.ctx || !this.isUnlocked || !this.shiftClunkBuffer) return;
    try {
      const now = this.ctx.currentTime;
      const src = this.ctx.createBufferSource();
      src.buffer = this.shiftClunkBuffer;

      const clunkGain = this.ctx.createGain();
      const safeVol = clamp(volume, 0.0, 0.30);
      clunkGain.gain.setValueAtTime(safeVol, now);

      src.connect(clunkGain);
      clunkGain.connect(this.sfxBus ?? this.masterGain!);
      src.start();
    } catch {
      // Audio playback failsafe
    }
  }

  /** Alias for playShiftClunk for API compatibility */
  public triggerShiftClunk(volume: number = 0.30): void {
    this.playShiftClunk(volume);
  }

  /**
   * Real-time update binding vehicle RPM, throttle load, shift status, and speed to audio parameters.
   * Updates existing Web Audio nodes strictly using automation methods without restarting buffers.
   * Supports both positional arguments and object-options format.
   */
  public update(
    engineRpmOrOptions: number | {
      engineRpm: number;
      throttle?: number;
      dt?: number;
      isShifting?: boolean;
      speedKmh?: number;
      rangeMode?: 'high' | 'low';
    },
    throttleArg: number = 0,
    dtArg: number = 0.016,
    isShiftingArg: boolean = false,
    speedKmhArg: number = 0,
    rangeModeArg: 'high' | 'low' = 'high'
  ): void {
    if (!this.ctx || !this.isUnlocked || this.isDisposed) return;

    let engineRpm: number;
    let throttle: number;
    let isShifting: boolean;
    let speedKmh: number;
    let rangeMode: 'high' | 'low';

    if (typeof engineRpmOrOptions === 'object' && engineRpmOrOptions !== null) {
      engineRpm = Number.isFinite(engineRpmOrOptions.engineRpm) ? engineRpmOrOptions.engineRpm : this.config.idleRpm;
      throttle = Number.isFinite(engineRpmOrOptions.throttle) ? engineRpmOrOptions.throttle! : 0;
      isShifting = Boolean(engineRpmOrOptions.isShifting);
      speedKmh = Number.isFinite(engineRpmOrOptions.speedKmh) ? engineRpmOrOptions.speedKmh! : 0;
      rangeMode = engineRpmOrOptions.rangeMode ?? 'high';
    } else {
      engineRpm = Number.isFinite(engineRpmOrOptions) ? engineRpmOrOptions : this.config.idleRpm;
      throttle = Number.isFinite(throttleArg) ? throttleArg : 0;
      isShifting = Boolean(isShiftingArg);
      speedKmh = Number.isFinite(speedKmhArg) ? speedKmhArg : 0;
      rangeMode = rangeModeArg;
    }

    this.currentRpm = engineRpm;
    this.currentThrottle = throttle;
    this.currentIsShifting = isShifting;
    this.currentSpeedKmh = speedKmh;
    this.currentRangeMode = rangeMode;

    const now = this.ctx.currentTime;
    const rpmFrac = clamp(
      (engineRpm - this.config.idleRpm) / (this.config.redlineRpm - this.config.idleRpm),
      0,
      1.15
    );

    // 1. Dynamic RPM Playback Rate Modulation with Micro-Wobble & 22% Shift Dip
    let targetPlaybackRate = calculatePlaybackRate(
      engineRpm,
      isShifting,
      rangeMode,
      throttle,
      speedKmh,
      this.config.idleRpm,
      this.config.redlineRpm,
      this.config.minPlaybackRate,
      this.config.maxPlaybackRate
    );

    // Micro-jitter: 0.5% random variation at 3.0 Hz to break up repetitive loop buzz
    const microWobble = 1.0 + 0.005 * Math.sin(now * 2.0 * Math.PI * 3.0);
    targetPlaybackRate *= microWobble;
    this.currentPlaybackRate = targetPlaybackRate;

    // Automation: 40ms time constant during shifts (smooth 0.18s transition), 50ms otherwise
    const pitchTimeConstant = isShifting ? 0.040 : 0.050;
    if (this.engineSource && this.engineSource.playbackRate) {
      this.engineSource.playbackRate.setTargetAtTime(targetPlaybackRate, now, pitchTimeConstant);
    }

    // 2. Dynamic LPF Modulation (180 Hz idle to 450 Hz high speed) with subtle flutter
    let targetCutoff = calculateLpfCutoff(
      engineRpm,
      throttle,
      speedKmh,
      rangeMode,
      this.config.minFilterCutoffHz,
      this.config.maxFilterCutoffHz
    );

    // Subtle 12 Hz flutter at 1.8 Hz to mask loop seam repetition
    const filterFlutter = 12.0 * Math.sin(now * 2.0 * Math.PI * 1.8);
    targetCutoff = clamp(targetCutoff + filterFlutter, this.config.minFilterCutoffHz, this.config.maxFilterCutoffHz);
    this.currentFilterCutoff = targetCutoff;

    if (this.engineFilter) {
      this.engineFilter.frequency.setTargetAtTime(targetCutoff, now, 0.050);
    }

    // 3. Master Engine Gain: baseline floor 0.28, max ceiling 0.38, duck by 15% during shifting
    const engineGain = calculateEngineGain(
      throttle,
      rpmFrac,
      isShifting,
      this.config.idleGainFloor,
      this.config.masterVolume
    );

    const gainTimeConstant = isShifting ? 0.035 : 0.050;
    if (this.engineBus) {
      this.engineBus.gain.setTargetAtTime(engineGain, now, gainTimeConstant);
    }

    // 4. LOW Range Gear-Mesh Whine (120 Hz to 220 Hz bandpass noise, silent in HIGH)
    if (this.gearMeshBpf && this.gearMeshGain) {
      const mesh = calculateLowRangeGearMesh(speedKmh, rangeMode);
      if (mesh.active) {
        this.gearMeshBpf.frequency.setTargetAtTime(mesh.centerFreqHz, now, 0.04);
        this.gearMeshGain.gain.setTargetAtTime(mesh.volume, now, 0.04);
      } else {
        this.gearMeshGain.gain.setTargetAtTime(0.0, now, 0.04);
      }
    }

    this.masterPeakGain = this.config.masterVolume * engineGain;
  }

  /**
   * Graceful teardown of audio context and audio resources.
   */
  public dispose(): void {
    if (this.isDisposed) return;
    this.isDisposed = true;

    try {
      if (this.engineSource) {
        this.engineSource.stop();
        this.engineSource.disconnect();
        this.engineSource = null;
      }
      if (this.gearMeshNoiseSource) {
        this.gearMeshNoiseSource.stop();
        this.gearMeshNoiseSource.disconnect();
        this.gearMeshNoiseSource = null;
      }
      if (this.masterGain) {
        this.masterGain.disconnect();
        this.masterGain = null;
      }
      if (this.ctx) {
        this.ctx.close();
        this.ctx = null;
      }
    } catch {
      // Cleanup failsafe
    }
  }

  public getContextState(): string {
    return this.ctx ? this.ctx.state : 'uninitialized';
  }

  public isAudioUnlocked(): boolean {
    return this.isUnlocked;
  }

  public isSampleLoaded(): boolean {
    return this.isBufferLoaded;
  }

  public getPlaybackRate(): number {
    return this.currentPlaybackRate;
  }

  public getFilterCutoff(): number {
    return this.currentFilterCutoff;
  }
}

// Dual export aliases for seamless import across codebase
export { VehicleAudio as VehicleAudioSystem };
export default VehicleAudio;
