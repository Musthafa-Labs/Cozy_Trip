/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import test from 'node:test';
import assert from 'node:assert/strict';
import {
  DEFAULT_AUDIO_CONFIG,
  TRACTOR_HARMONIC_COEFFICIENTS,
  calculateEffectiveRpm,
  calculateFundamentalHz,
  calculateSubBassHz,
  calculateLpfCutoffHz,
  calculateValveClatterCenterFreqHz,
  calculateTCaseWhine,
  calculateEngineVolume,
} from './audio';

test('1. Low-RPM Fundamental Pulse (18.3 Hz at 550 RPM to 233.3 Hz at 7000 RPM)', () => {
  const idleFundamental = calculateFundamentalHz(550, 550, 7000, 18.3, 233.3);
  assert.equal(Math.round(idleFundamental * 10) / 10, 18.3);

  const redlineFundamental = calculateFundamentalHz(7000, 550, 7000, 18.3, 233.3);
  assert.equal(Math.round(redlineFundamental * 10) / 10, 233.3);

  // Mid-range tractor check (e.g. 3775 RPM)
  const midFundamental = calculateFundamentalHz(3775, 550, 7000, 18.3, 233.3);
  assert.ok(midFundamental > 120 && midFundamental < 130);
});

test('2. Deep Exhaust Compression Sub-Bass is 1 octave below fundamental (9.15 Hz to 116.65 Hz)', () => {
  const idleSubBass = calculateSubBassHz(18.3);
  assert.equal(Math.round(idleSubBass * 100) / 100, 9.15);

  const redlineSubBass = calculateSubBassHz(233.3);
  assert.equal(Math.round(redlineSubBass * 100) / 100, 116.65);
});

test('3. Ultra-Muffled Dual LPF Cascade Cutoff is strictly bounded between 140 Hz and 350 Hz', () => {
  const idleCutoff = calculateLpfCutoffHz(0, 140.0, 350.0);
  assert.equal(idleCutoff, 140.0);

  const redlineCutoff = calculateLpfCutoffHz(1.0, 140.0, 350.0);
  assert.equal(redlineCutoff, 350.0);

  // Boundary clamping
  assert.equal(calculateLpfCutoffHz(-0.5, 140.0, 350.0), 140.0);
  assert.equal(calculateLpfCutoffHz(1.5, 140.0, 350.0), 350.0);
});

test('4. Tractor PeriodicWave Table Harmonics: dominant 1st (1.0) and 2nd (0.7)', () => {
  const expected = [1.0, 0.70, 0.10, 0.015, 0.002, 0.0005];
  assert.deepEqual(Array.from(TRACTOR_HARMONIC_COEFFICIENTS), expected);
  assert.equal(TRACTOR_HARMONIC_COEFFICIENTS[0], 1.0);
  assert.equal(TRACTOR_HARMONIC_COEFFICIENTS[1], 0.70);
  for (let i = 2; i < expected.length; i++) {
    assert.ok(TRACTOR_HARMONIC_COEFFICIENTS[i] < TRACTOR_HARMONIC_COEFFICIENTS[i - 1]);
  }
});

test('5. Low-Speed & Reverse Volume Floor maintains strict 0.78 minimum gain', () => {
  // Stationary idle (0 throttle, 0 RPM frac)
  const idleVol = calculateEngineVolume(0, 0);
  assert.equal(idleVol, 0.78);

  // Reverse / low crawl with full reverse throttle (-1.0)
  const revVol = calculateEngineVolume(-1.0, 0.05);
  assert.ok(revVol >= 0.78);
  assert.ok(revVol > 0.95);

  // Forward throttle
  const fwdVol = calculateEngineVolume(0.5, 0.2);
  assert.ok(fwdVol >= 0.78);
});

test('6. Mechanical Rocker & Injection Clatter center frequency is around 950 Hz', () => {
  const idleClatter = calculateValveClatterCenterFreqHz(0);
  assert.ok(idleClatter >= 930 && idleClatter <= 970);

  const redlineClatter = calculateValveClatterCenterFreqHz(1.0);
  assert.ok(redlineClatter >= 930 && redlineClatter <= 970);
});

test('7. LOW Range Gear-Mesh Whine scales 110 Hz -> 220 Hz (no laser/beam tones)', () => {
  // Silent when stationary
  const stationaryLow = calculateTCaseWhine(0, 'low');
  assert.equal(stationaryLow.active, false);
  assert.equal(stationaryLow.volume, 0);

  // Silent in HIGH range even at high speed
  const highwayHigh = calculateTCaseWhine(60, 'high');
  assert.equal(highwayHigh.active, false);
  assert.equal(highwayHigh.volume, 0);

  // Low range crawl
  const crawlLow = calculateTCaseWhine(1.0, 'low');
  assert.equal(crawlLow.active, true);
  assert.ok(crawlLow.frequencyHz >= 110 && crawlLow.frequencyHz <= 120);

  // Low range max speed (~45 km/h)
  const topLow = calculateTCaseWhine(45, 'low');
  assert.equal(topLow.active, true);
  assert.equal(Math.round(topLow.frequencyHz), 220);
});

test('8. Gear Shift RPM Dip applies an exact 20% frequency dip during clutch disengagement', () => {
  const normalRpm = 3000;
  const shiftingRpm = calculateEffectiveRpm(normalRpm, true);
  assert.equal(shiftingRpm, 2400); // 3000 * 0.80

  const steadyRpm = calculateEffectiveRpm(normalRpm, false);
  assert.equal(steadyRpm, 3000);
});
