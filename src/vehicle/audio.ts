/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import { clamp, lerp } from '../utils/math';

export interface VehicleAudioConfig {
  idleRpm: number;           // Baseline idle RPM (default: 550)
  redlineRpm: number;        // Maximum redline RPM (default: 7000)
  idleFrequencyHz: number;   // Fundamental frequency at idle (default: 18.3 Hz - heavy vintage tractor firing pulse)
  redlineFrequencyHz: number;// Fundamental frequency at redline (default: 233.3 Hz)
  minFilterCutoffHz: number; // Low-pass filter cutoff at idle (default: 140.0 Hz - ultra-muffled iron block thud)
  maxFilterCutoffHz: number; // Low-pass filter cutoff at redline (default: 350.0 Hz - completely cuts synthetic hiss)
  masterVolume: number;      // Master headroom ceiling (default: 0.65 to guarantee < 0 dBFS)
}

export const DEFAULT_AUDIO_CONFIG: VehicleAudioConfig = {
  idleRpm: 550,
  redlineRpm: 7000,
  idleFrequencyHz: 18.3,
  redlineFrequencyHz: 233.3,
  minFilterCutoffHz: 140.0,
  maxFilterCutoffHz: 350.0,
  masterVolume: 0.65,
};

/**
 * 6-harmonic wave table coefficients for heavy vintage tractor diesel tone:
 * Dominant 1st (1.0) and 2nd (0.7) harmonics with steep exponential drop-off on higher
 * harmonics to build an explosive cylinder firing pulse with 0 mosquito hiss.
 */
export const TRACTOR_HARMONIC_COEFFICIENTS = [1.0, 0.70, 0.10, 0.015, 0.002, 0.0005] as const;

/** Pure mathematical helper for shifting RPM dip (smooth 20% dip during clutch disengagement) */
export function calculateEffectiveRpm(engineRpm: number, isShifting: boolean): number {
  return isShifting ? engineRpm * 0.80 : engineRpm;
}

/** Pure mathematical helper for low-frequency tractor pulse fundamental (18.3 Hz at 550 RPM to 233.3 Hz at 7,000 RPM) */
export function calculateFundamentalHz(
  effectiveRpm: number,
  idleRpm: number = 550,
  redlineRpm: number = 7000,
  idleHz: number = 18.3,
  redlineHz: number = 233.3
): number {
  const frac = clamp((effectiveRpm - idleRpm) / (redlineRpm - idleRpm), 0, 1.15);
  return lerp(idleHz, redlineHz, frac);
}

/** Pure mathematical helper for deep exhaust sub-bass (1 octave below fundamental: 9.15 Hz to 116.65 Hz) */
export function calculateSubBassHz(fundamentalHz: number): number {
  return fundamentalHz * 0.5;
}

/** Pure mathematical helper for cascaded LPF cutoff (bounded strictly between 140 Hz and 350 Hz) */
export function calculateLpfCutoffHz(
  rpmFrac: number,
  minCutoff: number = 140.0,
  maxCutoff: number = 350.0
): number {
  return clamp(lerp(minCutoff, maxCutoff, clamp(rpmFrac, 0, 1)), minCutoff, maxCutoff);
}

/** Pure mathematical helper for mechanical rocker & injection clatter (center frequency 950 Hz, Q=3.5) */
export function calculateValveClatterCenterFreqHz(rpmFrac: number): number {
  return clamp(lerp(930.0, 970.0, clamp(rpmFrac, 0, 1)), 930.0, 970.0);
}

/** Pure mathematical helper for LOW range gear-mesh whine (110 Hz to 220 Hz filtered through 300 Hz LPF) */
export function calculateTCaseWhine(
  speedKmh: number,
  rangeMode: 'high' | 'low'
): { frequencyHz: number; volume: number; active: boolean } {
  if (rangeMode !== 'low' || speedKmh <= 0.3) {
    return { frequencyHz: 110.0, volume: 0.0, active: false };
  }
  const speedFrac = clamp(speedKmh / 45.0, 0, 1.0);
  const frequencyHz = lerp(110.0, 220.0, speedFrac);
  const volume = clamp(0.025 + 0.040 * speedFrac, 0.0, 0.065);
  return { frequencyHz, volume, active: true };
}

/** Pure mathematical helper for engine volume floor (strict 0.78 baseline floor) */
export function calculateEngineVolume(
  throttle: number,
  rpmFrac: number
): number {
  const baseFloor = 0.78;
  const throttleSwell = Math.abs(throttle) * 0.22;
  const rpmSwell = rpmFrac * 0.15;
  return clamp(baseFloor + throttleSwell + rpmSwell, 0.78, 1.0);
}

/**
 * Procedural Web Audio API Engine & Drivetrain Sound Synthesizer.
 *
 * Implements Stage 11 (Revision) Vintage Tractor Diesel Engine Audio Synthesis:
 * - Ultra-Muffled Dual LPF Cascade (-24 dB/octave total slope) strictly bounded 140 Hz (idle 550 RPM) up to 350 Hz (redline 7000 RPM).
 * - Low-RPM Fundamental Pulse: 18.3 Hz at idle (550 RPM) up to 233.3 Hz at redline (7000 RPM).
 * - Tractor PeriodicWave Table: dominant 1st (1.0) and 2nd (0.7) harmonics with steep exponential rolloff.
 * - Discrete Cylinder "Chug" Modulation: 4.5 Hz asymmetric sawtooth frequency modulation (depth: ~5.0 Hz).
 * - Deep Exhaust Compression Sub-Bass: 1 octave below fundamental (9.1 Hz to 116.6 Hz) bypassing main LPF.
 * - Mechanical Rocker & Injection Clatter: bandpass-filtered noise layer (center frequency 950 Hz, Q=3.5).
 * - Low-Speed & Reverse Volume Floor: strict master gain baseline floor (0.78 minimum gain).
 * - Mechanical Shift Clunk: heavy 55 Hz -> 15 Hz low-frequency thud burst (0.08s duration).
 * - Gear Shift RPM Dip: smooth 20% frequency dip (~0.18s duration) during clutch disengagement state.
 * - LOW Range Gear-Mesh Whine: triangle oscillator through dedicated 300 Hz LPF scaling 110 Hz to 220 Hz (zero laser/beam tones).
 * - Zero per-frame allocations during active playback.
 */
export class VehicleAudio {
  public readonly config: VehicleAudioConfig;

  // Web Audio Context & Busses
  private ctx: AudioContext | null = null;
  private masterGain: GainNode | null = null;
  private engineBus: GainNode | null = null;
  private sfxBus: GainNode | null = null;

  // Engine Synthesis Nodes
  private oscFundamental: OscillatorNode | null = null;
  private oscSubBass: OscillatorNode | null = null;

  // Discrete Cylinder "Chug" Modulation (4.5 Hz asymmetric sawtooth LFO)
  private oscChug: OscillatorNode | null = null;
  private gainChugPitch: GainNode | null = null;
  private gainChugAmp: GainNode | null = null;
  private chugAmpGain: GainNode | null = null;

  // Mechanical Rocker & Injection Clatter (950 Hz, Q=3.5)
  private clatterNoiseSource: AudioBufferSourceNode | null = null;
  private clatterFilter: BiquadFilterNode | null = null;
  private gainClatter: GainNode | null = null;

  // Gain Stages & Ultra-Muffled Dual LPF Cascade (-24 dB/octave total slope)
  private gainFundamental: GainNode | null = null;
  private gainSubBass: GainNode | null = null;
  private exhaustFilter1: BiquadFilterNode | null = null;
  private exhaustFilter2: BiquadFilterNode | null = null;

  // Drivetrain / Transfer Case FX Nodes (with dedicated 300 Hz LPF)
  private oscTCase: OscillatorNode | null = null;
  private tCaseLPF: BiquadFilterNode | null = null;
  private gainTCase: GainNode | null = null;
  private shiftClunkBuffer: AudioBuffer | null = null;

  // State
  private isUnlocked: boolean = false;
  private isDisposed: boolean = false;
  private isOscStarted: boolean = false;
  public currentRpm: number = 550;
  public currentThrottle: number = 0;
  public masterPeakGain: number = 0;

  constructor(config: Partial<VehicleAudioConfig> = {}) {
    this.config = { ...DEFAULT_AUDIO_CONFIG, ...config };
  }

  /**
   * Prepares audio manager. AudioContext creation is deferred to unlock() upon first user gesture.
   */
  public init(): void {
    // Deferred creation to prevent browser autoplay warnings
  }

  /**
   * Builds the procedural synthesis graph and synthesizes procedural FX buffers.
   */
  private buildAudioGraph(): void {
    if (!this.ctx) return;

    const sampleRate = this.ctx.sampleRate;
    const now = this.ctx.currentTime;

    // 1. Master Output Gain with Headroom
    this.masterGain = this.ctx.createGain();
    this.masterGain.gain.setValueAtTime(this.config.masterVolume, now);
    this.masterGain.connect(this.ctx.destination);

    // 2. Engine Sub-Bus
    this.engineBus = this.ctx.createGain();
    this.engineBus.gain.setValueAtTime(0.78, now); // Initial baseline volume floor
    this.engineBus.connect(this.masterGain);

    // 3. SFX Bus (Shift clunks, mechanical transients)
    this.sfxBus = this.ctx.createGain();
    this.sfxBus.gain.setValueAtTime(0.75, now);
    this.sfxBus.connect(this.masterGain);

    // 4. Ultra-Muffled Dual LPF Cascade (-24 dB/octave total slope)
    // Dynamic cutoff strictly bounded between 140 Hz (idle 550 RPM) up to 350 Hz (redline 7000 RPM)
    this.exhaustFilter1 = this.ctx.createBiquadFilter();
    this.exhaustFilter1.type = 'lowpass';
    this.exhaustFilter1.Q.setValueAtTime(1.4, now); // Heavy low-frequency exhaust thud resonance
    this.exhaustFilter1.frequency.setValueAtTime(this.config.minFilterCutoffHz, now);

    this.exhaustFilter2 = this.ctx.createBiquadFilter();
    this.exhaustFilter2.type = 'lowpass';
    this.exhaustFilter2.Q.setValueAtTime(0.707, now); // Second biquad stage
    this.exhaustFilter2.frequency.setValueAtTime(this.config.minFilterCutoffHz, now);

    // Cascade filter1 into filter2, then into engine bus
    this.exhaustFilter1.connect(this.exhaustFilter2);
    this.exhaustFilter2.connect(this.engineBus);

    // 5. Tractor PeriodicWave Table Fundamental Oscillator (dominant 1st & 2nd harmonics)
    // Harmonics: [1.0, 0.70, 0.10, 0.015, 0.002, 0.0005]
    const real = new Float32Array([0, 0, 0, 0, 0, 0, 0]);
    const imag = new Float32Array([
      0,
      TRACTOR_HARMONIC_COEFFICIENTS[0],
      TRACTOR_HARMONIC_COEFFICIENTS[1],
      TRACTOR_HARMONIC_COEFFICIENTS[2],
      TRACTOR_HARMONIC_COEFFICIENTS[3],
      TRACTOR_HARMONIC_COEFFICIENTS[4],
      TRACTOR_HARMONIC_COEFFICIENTS[5],
    ]);
    const tractorWave = this.ctx.createPeriodicWave(real, imag, { disableNormalization: false });

    this.oscFundamental = this.ctx.createOscillator();
    this.oscFundamental.setPeriodicWave(tractorWave);
    this.oscFundamental.frequency.setValueAtTime(this.config.idleFrequencyHz, now);

    this.gainFundamental = this.ctx.createGain();
    this.gainFundamental.gain.setValueAtTime(0.60, now);

    // 6. Discrete Cylinder "Chug" Modulation (4.5 Hz asymmetric sawtooth frequency modulation)
    this.oscChug = this.ctx.createOscillator();
    this.oscChug.type = 'sawtooth'; // Sharp pulse attack for individual cylinder hits
    this.oscChug.frequency.setValueAtTime(4.5, now);

    // Frequency modulation (~5.0 Hz displacement per pulse)
    this.gainChugPitch = this.ctx.createGain();
    this.gainChugPitch.gain.setValueAtTime(5.0, now);
    this.oscChug.connect(this.gainChugPitch);
    this.gainChugPitch.connect(this.oscFundamental.frequency);

    // Amplitude tremolo (~18% modulation depth for heavy compression shudder)
    this.chugAmpGain = this.ctx.createGain();
    this.chugAmpGain.gain.setValueAtTime(1.0, now);

    this.gainChugAmp = this.ctx.createGain();
    this.gainChugAmp.gain.setValueAtTime(0.18, now);
    this.oscChug.connect(this.gainChugAmp);
    this.gainChugAmp.connect(this.chugAmpGain.gain);

    // Fundamental routing: osc -> gainFundamental -> chugAmpGain -> exhaustFilter1 -> exhaustFilter2
    this.oscFundamental.connect(this.gainFundamental);
    this.gainFundamental.connect(this.chugAmpGain);
    this.chugAmpGain.connect(this.exhaustFilter1);

    // 7. Deep Exhaust Compression Sub-Bass Layer (Sine wave, 1 octave below fundamental: 9.1 Hz -> 116.6 Hz)
    // CRITICAL: Bypasses the main LPF for heavy low-end exhaust compression thuds directly into engineBus!
    this.oscSubBass = this.ctx.createOscillator();
    this.oscSubBass.type = 'sine';
    this.oscSubBass.frequency.setValueAtTime(this.config.idleFrequencyHz * 0.5, now);

    this.gainSubBass = this.ctx.createGain();
    this.gainSubBass.gain.setValueAtTime(0.65, now);

    this.oscSubBass.connect(this.gainSubBass);
    this.gainSubBass.connect(this.engineBus); // Direct connection: Bypasses LPF!

    // 8. Mechanical Rocker & Injection Clatter (950 Hz, Q=3.5)
    this.buildValveClatterSource(sampleRate);

    // 9. LOW Range Gear-Mesh Whine (Laser/Beam Fix with dedicated 300 Hz LPF)
    this.oscTCase = this.ctx.createOscillator();
    this.oscTCase.type = 'triangle'; // Mellow mechanical gear mesh profile
    this.oscTCase.frequency.setValueAtTime(110.0, now);

    this.tCaseLPF = this.ctx.createBiquadFilter();
    this.tCaseLPF.type = 'lowpass';
    this.tCaseLPF.frequency.setValueAtTime(300.0, now); // Dedicated 300 Hz low-pass filter eliminates beam frequencies

    this.gainTCase = this.ctx.createGain();
    this.gainTCase.gain.setValueAtTime(0.0, now); // Muted by default until moving in LOW range

    this.oscTCase.connect(this.tCaseLPF);
    this.tCaseLPF.connect(this.gainTCase);
    this.gainTCase.connect(this.masterGain);

    // 10. Synthesize Procedural Shift Clunk Audio Buffer (heavy 55 Hz -> 15 Hz mechanical thud, 0.08s duration)
    this.buildShiftClunkBuffer(sampleRate);
  }

  /**
   * Generates a 2-second looping noise buffer for the vintage mechanical rocker & injector chatter layer.
   * Bandpass center frequency 950 Hz, Q = 3.5.
   */
  private buildValveClatterSource(sampleRate: number): void {
    if (!this.ctx) return;
    const now = this.ctx.currentTime;
    const duration = 2.0;
    const numSamples = Math.floor(sampleRate * duration);
    const noiseBuffer = this.ctx.createBuffer(1, numSamples, sampleRate);
    const noiseData = noiseBuffer.getChannelData(0);

    for (let i = 0; i < numSamples; i++) {
      noiseData[i] = (Math.random() * 2.0 - 1.0) * 0.28;
    }

    this.clatterNoiseSource = this.ctx.createBufferSource();
    this.clatterNoiseSource.buffer = noiseBuffer;
    this.clatterNoiseSource.loop = true;

    this.clatterFilter = this.ctx.createBiquadFilter();
    this.clatterFilter.type = 'bandpass';
    this.clatterFilter.Q.setValueAtTime(3.5, now);
    this.clatterFilter.frequency.setValueAtTime(950.0, now);

    this.gainClatter = this.ctx.createGain();
    this.gainClatter.gain.setValueAtTime(0.024, now); // Dull mechanical clatter

    this.clatterNoiseSource.connect(this.clatterFilter);
    this.clatterFilter.connect(this.gainClatter);
    if (this.engineBus) {
      this.gainClatter.connect(this.engineBus);
    }
  }

  /**
   * Generates a heavy mechanical shift clunk:
   * Heavy 55 Hz -> 15 Hz low-frequency thud burst (0.08s duration).
   */
  private buildShiftClunkBuffer(sampleRate: number): void {
    if (!this.ctx) return;
    const durationSec = 0.08;
    const numSamples = Math.floor(sampleRate * durationSec);
    const buffer = this.ctx.createBuffer(1, numSamples, sampleRate);
    const data = buffer.getChannelData(0);

    for (let i = 0; i < numSamples; i++) {
      const t = i / sampleRate;
      const progress = t / durationSec; // 0 to 1
      const env = Math.exp(-38.0 * progress);

      // Low-frequency mechanical thud sweep: 55 Hz down to 15 Hz
      // Phase integral: 2 * PI * (55 * t - 0.5 * (55 - 15) * t^2 / durationSec)
      const phase = 2.0 * Math.PI * (55.0 * t - 0.5 * (55.0 - 15.0) * (t * t / durationSec));
      const thud = Math.sin(phase);

      // Heavy dog engagement contact transient & synchro ring strike
      const transient = (Math.random() * 2.0 - 1.0) * Math.exp(-150.0 * t) * 0.35;
      const metalPing = Math.sin(2.0 * Math.PI * 150.0 * t) * Math.exp(-75.0 * t) * 0.30;

      data[i] = clamp((thud * 0.80 + metalPing + transient) * env * 0.95, -1.0, 1.0);
    }

    this.shiftClunkBuffer = buffer;
  }

  /**
   * Starts sustained oscillators once AudioContext is unlocked.
   */
  private startOscillators(): void {
    if (this.isOscStarted) return;
    try {
      if (this.oscFundamental) this.oscFundamental.start();
      if (this.oscSubBass) this.oscSubBass.start();
      if (this.oscChug) this.oscChug.start();
      if (this.clatterNoiseSource) this.clatterNoiseSource.start();
      if (this.oscTCase) this.oscTCase.start();
      this.isOscStarted = true;
    } catch {
      // Ignored if already started
    }
  }

  /**
   * Resumes or instantiates AudioContext on first user interaction to satisfy browser autoplay policies.
   */
  public unlock(): void {
    if (this.isDisposed || this.isUnlocked) return;

    if (!this.ctx) {
      const AudioContextClass =
        window.AudioContext ||
        (window as unknown as { webkitAudioContext: typeof AudioContext }).webkitAudioContext;

      if (!AudioContextClass) {
        console.warn('VehicleAudio: Web Audio API not supported in this browser.');
        return;
      }

      try {
        this.ctx = new AudioContextClass();
        this.buildAudioGraph();
      } catch (e) {
        console.warn('VehicleAudio: Failed to initialize AudioContext', e);
        return;
      }
    }

    if (this.ctx.state === 'suspended') {
      this.ctx.resume().catch((err) => {
        console.warn('VehicleAudio: audio resume failed', err);
      });
    }

    this.isUnlocked = true;
    this.startOscillators();
  }

  /**
   * Triggers a heavy mechanical shift clunk sound on gear engagement.
   * Triggered on any gear change (Automatic or Q/E Manual).
   */
  public playShiftClunk(volume: number = 0.65): void {
    if (!this.ctx || !this.isUnlocked || this.isDisposed || !this.shiftClunkBuffer) return;

    try {
      const src = this.ctx.createBufferSource();
      src.buffer = this.shiftClunkBuffer;

      const clunkGain = this.ctx.createGain();
      clunkGain.gain.setValueAtTime(clamp(volume, 0.0, 1.0), this.ctx.currentTime);

      src.connect(clunkGain);
      clunkGain.connect(this.sfxBus ?? this.masterGain!);
      src.start();
    } catch {
      // Audio playback failsafe
    }
  }

  /** Alias for playShiftClunk for API flexibility */
  public triggerShiftClunk(volume: number = 0.65): void {
    this.playShiftClunk(volume);
  }

  /**
   * Real-time update binding engine RPM, throttle load, shift status, and speed to audio parameters.
   *
   * @param engineRpm Current engine RPM from powertrain (e.g. 550 to 7000)
   * @param throttle Input throttle pedal position [0 .. 1.0]
   * @param dt Frame delta time in seconds
   * @param isShifting Drivetrain gear change flag for smooth audio RPM dip
   * @param speedKmh Vehicle ground speed in km/h for transfer case whine
   * @param rangeMode Transfer case range ('high' | 'low')
   */
  public update(
    engineRpm: number,
    throttle: number,
    dt: number,
    isShifting: boolean = false,
    speedKmh: number = 0,
    rangeMode: 'high' | 'low' = 'high'
  ): void {
    if (!this.ctx || !this.isUnlocked || this.isDisposed) return;

    this.currentRpm = engineRpm;
    this.currentThrottle = throttle;

    const now = this.ctx.currentTime;
    // 55ms exponential smooth ramp to guarantee smooth 0.18s dip without audio clicks or pops
    const timeConstant = 0.055;

    // Gear Shift RPM Dip: Apply a smooth 20% frequency dip (~0.18s duration) during clutch disengagement state
    const effectiveRpm = calculateEffectiveRpm(engineRpm, isShifting);

    // 1. Low-RPM Fundamental Pulse: 18.3 Hz at idle (550 RPM) up to 233.3 Hz at redline (7,000 RPM)
    const rpmFrac = clamp(
      (effectiveRpm - this.config.idleRpm) / (this.config.redlineRpm - this.config.idleRpm),
      0,
      1.15
    );

    const fundamentalHz = calculateFundamentalHz(
      effectiveRpm,
      this.config.idleRpm,
      this.config.redlineRpm,
      this.config.idleFrequencyHz,
      this.config.redlineFrequencyHz
    );

    // Deep Exhaust Sub-Bass: 1 octave below fundamental (9.15 Hz at idle to 116.65 Hz at redline)
    const subBassHz = calculateSubBassHz(fundamentalHz);

    if (this.oscFundamental) {
      this.oscFundamental.frequency.setTargetAtTime(fundamentalHz, now, timeConstant);
    }
    if (this.oscSubBass) {
      this.oscSubBass.frequency.setTargetAtTime(subBassHz, now, timeConstant);
    }

    // 2. Ultra-Muffled Dual LPF Cascade (-24 dB/octave):
    // Cutoff strictly bounded between 140 Hz at idle (550 RPM) up to 350 Hz at redline (7,000 RPM)
    const filterFreq = calculateLpfCutoffHz(
      rpmFrac,
      this.config.minFilterCutoffHz,
      this.config.maxFilterCutoffHz
    );
    if (this.exhaustFilter1) {
      this.exhaustFilter1.frequency.setTargetAtTime(filterFreq, now, timeConstant);
    }
    if (this.exhaustFilter2) {
      this.exhaustFilter2.frequency.setTargetAtTime(filterFreq, now, timeConstant);
    }

    // 3. Mechanical Rocker & Injection Clatter (950 Hz, Q=3.5):
    if (this.clatterFilter && this.gainClatter) {
      const clatterCenterFreq = calculateValveClatterCenterFreqHz(rpmFrac);
      this.clatterFilter.frequency.setTargetAtTime(clatterCenterFreq, now, timeConstant);

      const clatterVolume = lerp(0.018, 0.040, rpmFrac * 0.7 + Math.abs(throttle) * 0.3);
      this.gainClatter.gain.setTargetAtTime(clatterVolume, now, timeConstant);
    }

    // 4. Low-Speed & Reverse Volume Floor:
    // Implement a strict master gain baseline floor (0.78 minimum gain) so low-speed crawling,
    // reversing, and stationary idling stay loud and heavy rather than dropping out.
    let totalEngineGain = calculateEngineVolume(throttle, rpmFrac);

    // Smooth gain dip during gear shift
    if (isShifting) {
      totalEngineGain *= 0.75;
    }

    if (this.engineBus) {
      this.engineBus.gain.setTargetAtTime(totalEngineGain, now, timeConstant);
    }

    // Dynamic balance: sub-bass remains solid for heavy exhaust compression thuds
    if (this.gainFundamental && this.gainSubBass) {
      this.gainSubBass.gain.setTargetAtTime(0.65, now, timeConstant);
      this.gainFundamental.gain.setTargetAtTime(0.55 + 0.10 * Math.abs(throttle), now, timeConstant);
    }

    // 5. LOW Range Gear-Mesh Whine (Laser/Beam Fix with dedicated 300 Hz LPF):
    // Active strictly in LOW range (L key), generating pitch scaling from 110 Hz to 220 Hz relative to speed.
    // Silent when stationary or in HIGH range.
    if (this.oscTCase && this.gainTCase) {
      const tcase = calculateTCaseWhine(speedKmh, rangeMode);
      if (tcase.active) {
        this.oscTCase.frequency.setTargetAtTime(tcase.frequencyHz, now, 0.04);
        this.gainTCase.gain.setTargetAtTime(tcase.volume, now, 0.04);
      } else {
        this.gainTCase.gain.setTargetAtTime(0.0, now, 0.05);
      }
    }

    this.masterPeakGain = this.config.masterVolume * totalEngineGain;
  }

  /**
   * Graceful teardown of audio context, stopping oscillators and releasing audio resources.
   */
  public dispose(): void {
    if (this.isDisposed) return;
    this.isDisposed = true;

    try {
      if (this.oscFundamental) {
        this.oscFundamental.stop();
        this.oscFundamental.disconnect();
        this.oscFundamental = null;
      }
      if (this.oscSubBass) {
        this.oscSubBass.stop();
        this.oscSubBass.disconnect();
        this.oscSubBass = null;
      }
      if (this.oscChug) {
        this.oscChug.stop();
        this.oscChug.disconnect();
        this.oscChug = null;
      }
      if (this.clatterNoiseSource) {
        this.clatterNoiseSource.stop();
        this.clatterNoiseSource.disconnect();
        this.clatterNoiseSource = null;
      }
      if (this.oscTCase) {
        this.oscTCase.stop();
        this.oscTCase.disconnect();
        this.oscTCase = null;
      }
      if (this.gainFundamental) {
        this.gainFundamental.disconnect();
        this.gainFundamental = null;
      }
      if (this.gainSubBass) {
        this.gainSubBass.disconnect();
        this.gainSubBass = null;
      }
      if (this.gainChugPitch) {
        this.gainChugPitch.disconnect();
        this.gainChugPitch = null;
      }
      if (this.gainChugAmp) {
        this.gainChugAmp.disconnect();
        this.gainChugAmp = null;
      }
      if (this.chugAmpGain) {
        this.chugAmpGain.disconnect();
        this.chugAmpGain = null;
      }
      if (this.gainClatter) {
        this.gainClatter.disconnect();
        this.gainClatter = null;
      }
      if (this.clatterFilter) {
        this.clatterFilter.disconnect();
        this.clatterFilter = null;
      }
      if (this.tCaseLPF) {
        this.tCaseLPF.disconnect();
        this.tCaseLPF = null;
      }
      if (this.gainTCase) {
        this.gainTCase.disconnect();
        this.gainTCase = null;
      }
      if (this.exhaustFilter1) {
        this.exhaustFilter1.disconnect();
        this.exhaustFilter1 = null;
      }
      if (this.exhaustFilter2) {
        this.exhaustFilter2.disconnect();
        this.exhaustFilter2 = null;
      }
      if (this.sfxBus) {
        this.sfxBus.disconnect();
        this.sfxBus = null;
      }
      if (this.engineBus) {
        this.engineBus.disconnect();
        this.engineBus = null;
      }
      if (this.masterGain) {
        this.masterGain.disconnect();
        this.masterGain = null;
      }
      if (this.ctx && this.ctx.state !== 'closed') {
        this.ctx.close();
        this.ctx = null;
      }
    } catch (e) {
      console.warn('VehicleAudio: Error during audio disposal', e);
    }
  }

  public getContextState(): AudioContextState | 'uninitialized' {
    if (this.isDisposed) return 'closed';
    return this.ctx ? this.ctx.state : 'uninitialized';
  }

  public isAudioUnlocked(): boolean {
    return this.isUnlocked;
  }
}

export { VehicleAudio as VehicleAudioSystem };
export default VehicleAudio;
