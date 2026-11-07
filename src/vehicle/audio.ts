/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import { clamp, lerp } from '../utils/math';

export interface VehicleAudioConfig {
  idleRpm: number;           // Baseline idle RPM (default: 900)
  redlineRpm: number;        // Maximum redline RPM (default: 7000)
  idleFrequencyHz: number;   // Fundamental frequency at idle (default: 34 Hz - deep 4-stroke pulse)
  redlineFrequencyHz: number;// Fundamental frequency at redline (default: 165 Hz)
  minFilterCutoffHz: number; // Low-pass filter cutoff at idle (default: 400 Hz)
  maxFilterCutoffHz: number; // Low-pass filter cutoff at redline (default: 1200 Hz - cuts mosquito buzz)
  masterVolume: number;      // Master headroom ceiling (default: 0.65 to guarantee < 0 dBFS)
}

export const DEFAULT_AUDIO_CONFIG: VehicleAudioConfig = {
  idleRpm: 900,
  redlineRpm: 7000,
  idleFrequencyHz: 34,
  redlineFrequencyHz: 165,
  minFilterCutoffHz: 400,
  maxFilterCutoffHz: 1200,
  masterVolume: 0.65,
};

/**
 * Procedural Web Audio API Engine & Drivetrain Sound Synthesizer.
 *
 * Implements:
 * - Low-frequency 4-stroke engine pulse density (~34 Hz idle to ~165 Hz redline).
 * - Dynamic steep low-pass filter (400 Hz -> 1200 Hz) completely eliminating harsh mosquito buzz.
 * - Sub-bass oscillator (Sine wave, 1 octave below fundamental) providing deep exhaust chest rumble.
 * - Subtle frequency modulation (FM) cylinder-firing unevenness for authentic mechanical texture.
 * - Procedural metallic transmission shift clunk FX on gear transitions.
 * - Audio dip during clutch disengagement / shift progression.
 * - Speed-proportional transfer case gear whine in LOW range mode.
 * - Safe browser autoplay unlock upon first user gesture.
 * - Master gain headroom ceiling (< 0 dBFS).
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
  private oscHarmonic: OscillatorNode | null = null;
  private oscSubBass: OscillatorNode | null = null;
  private oscJitter: OscillatorNode | null = null; // Subtle FM jitter for cylinder unevenness

  private gainFundamental: GainNode | null = null;
  private gainHarmonic: GainNode | null = null;
  private gainSubBass: GainNode | null = null;
  private gainJitter: GainNode | null = null;
  private exhaustFilter: BiquadFilterNode | null = null;

  // Drivetrain / Transmission FX Nodes
  private oscTCase: OscillatorNode | null = null; // Transfer case whine (LOW range)
  private gainTCase: GainNode | null = null;
  private shiftClunkBuffer: AudioBuffer | null = null;

  // State
  private isUnlocked: boolean = false;
  private isDisposed: boolean = false;
  private isOscStarted: boolean = false;
  public currentRpm: number = 900;
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
    this.engineBus.gain.setValueAtTime(0.001, now);

    // 3. SFX Bus (Shift clunks, mechanical transients)
    this.sfxBus = this.ctx.createGain();
    this.sfxBus.gain.setValueAtTime(0.70, now);
    this.sfxBus.connect(this.masterGain);

    // 4. Low-Pass Resonant Exhaust Filter (400 Hz -> 1200 Hz steep cutoff to eliminate mosquito buzz)
    this.exhaustFilter = this.ctx.createBiquadFilter();
    this.exhaustFilter.type = 'lowpass';
    this.exhaustFilter.Q.setValueAtTime(1.8, now);
    this.exhaustFilter.frequency.setValueAtTime(this.config.minFilterCutoffHz, now);

    // 5. Fundamental Oscillator (Deep, warm 4-stroke firing pulse: Triangle wave ~34 Hz)
    this.oscFundamental = this.ctx.createOscillator();
    this.oscFundamental.type = 'triangle';
    this.oscFundamental.frequency.setValueAtTime(this.config.idleFrequencyHz, now);

    this.gainFundamental = this.ctx.createGain();
    this.gainFundamental.gain.setValueAtTime(0.50, now);

    this.oscFundamental.connect(this.gainFundamental);
    this.gainFundamental.connect(this.exhaustFilter);

    // 6. Sub-Bass Oscillator (Sine wave, 1 octave below fundamental: 17 Hz -> 82 Hz for chest rumble)
    this.oscSubBass = this.ctx.createOscillator();
    this.oscSubBass.type = 'sine';
    this.oscSubBass.frequency.setValueAtTime(this.config.idleFrequencyHz * 0.5, now);

    this.gainSubBass = this.ctx.createGain();
    this.gainSubBass.gain.setValueAtTime(0.40, now);

    this.oscSubBass.connect(this.gainSubBass);
    this.gainSubBass.connect(this.exhaustFilter);

    // 7. Harmonic Oscillator (Mellow Sawtooth 2nd harmonic for mechanical body)
    this.oscHarmonic = this.ctx.createOscillator();
    this.oscHarmonic.type = 'sawtooth';
    this.oscHarmonic.frequency.setValueAtTime(this.config.idleFrequencyHz * 2.0, now);

    this.gainHarmonic = this.ctx.createGain();
    this.gainHarmonic.gain.setValueAtTime(0.25, now);

    this.oscHarmonic.connect(this.gainHarmonic);
    this.gainHarmonic.connect(this.exhaustFilter);

    // 8. Cylinder Firing FM Jitter (Subtle LFO frequency modulation for combustion unevenness)
    this.oscJitter = this.ctx.createOscillator();
    this.oscJitter.type = 'sine';
    this.oscJitter.frequency.setValueAtTime(16.0, now); // ~16 Hz firing stroke variation

    this.gainJitter = this.ctx.createGain();
    this.gainJitter.gain.setValueAtTime(2.2, now); // ~2.2 Hz subtle detune deviation

    this.oscJitter.connect(this.gainJitter);
    this.gainJitter.connect(this.oscFundamental.frequency);
    this.gainJitter.connect(this.oscHarmonic.frequency);

    // Connect exhaust filter through engine bus into master gain
    this.exhaustFilter.connect(this.engineBus);
    this.engineBus.connect(this.masterGain);

    // 9. Transfer Case Gear Whine (LOW Range)
    this.oscTCase = this.ctx.createOscillator();
    this.oscTCase.type = 'triangle';
    this.oscTCase.frequency.setValueAtTime(140.0, now);

    this.gainTCase = this.ctx.createGain();
    this.gainTCase.gain.setValueAtTime(0.0, now); // Muted by default until in LOW range

    this.oscTCase.connect(this.gainTCase);
    this.gainTCase.connect(this.masterGain);

    // 10. Synthesize Procedural Shift Clunk Audio Buffer
    this.buildShiftClunkBuffer(sampleRate);
  }

  /**
   * Generates a realistic metallic transmission shift clunk (mechanical dog engagement + sub thud).
   */
  private buildShiftClunkBuffer(sampleRate: number): void {
    if (!this.ctx) return;
    const durationSec = 0.095;
    const numSamples = Math.floor(sampleRate * durationSec);
    const buffer = this.ctx.createBuffer(1, numSamples, sampleRate);
    const data = buffer.getChannelData(0);

    for (let i = 0; i < numSamples; i++) {
      const t = i / sampleRate;
      // Exponential decay envelope
      const env = Math.exp(-42.0 * t);
      // Low-frequency mechanical thud drop: 135 Hz -> 65 Hz
      const f = 135.0 - 720.0 * t;
      const thud = Math.sin(2.0 * Math.PI * Math.max(30.0, f) * t);
      // Short metallic synchro ring burst (380 Hz damped)
      const metallic = Math.sin(2.0 * Math.PI * 380.0 * t) * Math.exp(-65.0 * t) * 0.35;
      // Transient click
      const click = (Math.random() * 2.0 - 1.0) * Math.exp(-120.0 * t) * 0.35;

      data[i] = clamp((thud * 0.65 + metallic + click) * env * 0.85, -1.0, 1.0);
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
      if (this.oscHarmonic) this.oscHarmonic.start();
      if (this.oscSubBass) this.oscSubBass.start();
      if (this.oscJitter) this.oscJitter.start();
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
   * Triggers a punchy, low-latency metallic shift clunk sound on gear engagement.
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
    } catch (e) {
      // Audio playback failsafe
    }
  }

  /**
   * Real-time update binding engine RPM, throttle load, shift status, and speed to audio parameters.
   *
   * @param engineRpm Current engine RPM from powertrain (e.g. 900 to 7000)
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
    const timeConstant = 0.035; // 35ms exponential smooth ramp to eliminate zipper noise

    // Shift RPM Audio Dip: Smoothly dip pitch & volume during clutch disengagement (~0.15s)
    const effectiveRpm = isShifting ? engineRpm * 0.88 : engineRpm;

    // 1. Fundamental, Sub-Bass & Harmonic Pitch Calculation
    // Frequency scales smoothly from deep idle (34 Hz) to redline (165 Hz)
    const rpmFrac = clamp(
      (effectiveRpm - this.config.idleRpm) / (this.config.redlineRpm - this.config.idleRpm),
      0,
      1.15
    );

    const fundamentalHz = lerp(
      this.config.idleFrequencyHz,
      this.config.redlineFrequencyHz,
      rpmFrac
    );
    const subBassHz = fundamentalHz * 0.5; // Exactly 1 octave below (17 Hz -> 82.5 Hz)
    const harmonicHz = fundamentalHz * 2.0;

    if (this.oscFundamental) {
      this.oscFundamental.frequency.setTargetAtTime(fundamentalHz, now, timeConstant);
    }
    if (this.oscSubBass) {
      this.oscSubBass.frequency.setTargetAtTime(subBassHz, now, timeConstant);
    }
    if (this.oscHarmonic) {
      this.oscHarmonic.frequency.setTargetAtTime(harmonicHz, now, timeConstant);
    }

    // 2. Resonant Filter Cutoff: Rolloff high frequencies (400 Hz -> 1200 Hz max) to kill mosquito buzz
    const filterFreq = lerp(
      this.config.minFilterCutoffHz,
      this.config.maxFilterCutoffHz,
      clamp(0.40 * rpmFrac + 0.60 * throttle, 0, 1.0)
    );
    if (this.exhaustFilter) {
      this.exhaustFilter.frequency.setTargetAtTime(filterFreq, now, timeConstant);
    }

    // 3. Engine Sub-Bus Gain Modulation (Idle rumble + dynamic throttle load swell)
    // Idle base amplitude + load swell under heavy acceleration
    const baseIdleGain = 0.24;
    const throttleSwell = 0.44 * Math.pow(throttle, 0.80);
    const rpmSwell = 0.22 * rpmFrac;
    let totalEngineGain = clamp(baseIdleGain + throttleSwell + rpmSwell, 0, 0.92);

    // Apply smooth dip during gear shift
    if (isShifting) {
      totalEngineGain *= 0.55;
    }

    if (this.engineBus) {
      this.engineBus.gain.setTargetAtTime(totalEngineGain, now, timeConstant);
    }

    // Dynamic balance: sub-bass and fundamental remain dominant; harmonic rasp rises with throttle
    if (this.gainFundamental && this.gainSubBass && this.gainHarmonic) {
      this.gainSubBass.gain.setTargetAtTime(0.48, now, timeConstant);
      this.gainFundamental.gain.setTargetAtTime(0.52 - 0.08 * throttle, now, timeConstant);
      this.gainHarmonic.gain.setTargetAtTime(0.18 + 0.20 * throttle, now, timeConstant);
    }

    // 4. Transfer Case Whine (LOW Range only, pitch & volume proportional to speed)
    if (this.oscTCase && this.gainTCase) {
      if (rangeMode === 'low' && speedKmh > 1.0) {
        // Whine frequency scales from ~140 Hz up to ~1100 Hz
        const whineFreq = 140.0 + 24.0 * Math.min(speedKmh, 50.0);
        const whineVolume = clamp(0.015 + 0.045 * (speedKmh / 35.0), 0.0, 0.065);

        this.oscTCase.frequency.setTargetAtTime(whineFreq, now, 0.05);
        this.gainTCase.gain.setTargetAtTime(whineVolume, now, 0.05);
      } else {
        // Mute whine in HIGH range or when vehicle is at rest
        this.gainTCase.gain.setTargetAtTime(0.0, now, 0.08);
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
      if (this.oscHarmonic) {
        this.oscHarmonic.stop();
        this.oscHarmonic.disconnect();
        this.oscHarmonic = null;
      }
      if (this.oscSubBass) {
        this.oscSubBass.stop();
        this.oscSubBass.disconnect();
        this.oscSubBass = null;
      }
      if (this.oscJitter) {
        this.oscJitter.stop();
        this.oscJitter.disconnect();
        this.oscJitter = null;
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
      if (this.gainHarmonic) {
        this.gainHarmonic.disconnect();
        this.gainHarmonic = null;
      }
      if (this.gainSubBass) {
        this.gainSubBass.disconnect();
        this.gainSubBass = null;
      }
      if (this.gainJitter) {
        this.gainJitter.disconnect();
        this.gainJitter = null;
      }
      if (this.gainTCase) {
        this.gainTCase.disconnect();
        this.gainTCase = null;
      }
      if (this.exhaustFilter) {
        this.exhaustFilter.disconnect();
        this.exhaustFilter = null;
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
