/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import test from 'node:test';
import assert from 'node:assert/strict';
import {
  DEFAULT_AUDIO_CONFIG,
  calculatePlaybackRate,
  calculateLpfCutoff,
  calculateEngineGain,
  calculateLowRangeGearMesh,
  calculateShiftClunkParams,
} from './audio';

test('1. Dynamic RPM Playback Rate Modulation (1.0x at 550 RPM to 2.2x at 7000 RPM)', () => {
  // Idle (550 RPM, 0 throttle, 0 speed)
  const idleRate = calculatePlaybackRate(550, false, 'high', 0, 0, 550, 7000, 1.0, 2.2);
  assert.equal(Math.round(idleRate * 100) / 100, 1.0);

  // Redline (7000 RPM)
  const redlineRate = calculatePlaybackRate(7000, false, 'high', 1.0, 100, 550, 7000, 1.0, 2.2);
  assert.equal(Math.round(redlineRate * 100) / 100, 2.2);

  // Mid-range (3775 RPM)
  const midRate = calculatePlaybackRate(3775, false, 'high', 0.5, 50, 550, 7000, 1.0, 2.2);
  assert.ok(midRate > 1.5 && midRate < 1.7);
});

test('2. Gear Shift State Machine: 22% playbackRate dip during clutch disengagement', () => {
  const normalRate = calculatePlaybackRate(3000, false, 'high', 0.8, 40, 550, 7000, 1.0, 2.2);
  const shiftingRate = calculatePlaybackRate(3000, true, 'high', 0.8, 40, 550, 7000, 1.0, 2.2);

  // shiftingRate must equal normalRate * 0.78 (22% drop)
  assert.equal(Math.round((shiftingRate / normalRate) * 100) / 100, 0.78);
  assert.ok(shiftingRate < normalRate);
});

test('3. Reverse & LOW Range State Handling', () => {
  // Reverse throttle (-1.0) produces steady engine rumble between 1.05x and 1.20x
  const revRate = calculatePlaybackRate(800, false, 'high', -1.0, -5.0, 550, 7000, 1.0, 2.2);
  assert.ok(revRate >= 1.05 && revRate <= 1.20);

  // LOW range compresses speed-to-RPM sweep (shorter gear acceleration bands)
  const highRate = calculatePlaybackRate(2500, false, 'high', 0.5, 20, 550, 7000, 1.0, 2.2);
  const lowRate = calculatePlaybackRate(2500, false, 'low', 0.5, 20, 550, 7000, 1.0, 2.2);
  assert.ok(lowRate > highRate, 'LOW range rev pitch should be higher for same RPM fraction');
});

test('4. Dynamic LPF Modulation (180 Hz at idle to 450 Hz at high speed)', () => {
  // Idle (550 RPM, 0 throttle, 0 speed)
  const idleCutoff = calculateLpfCutoff(550, 0, 0, 'high', 180.0, 450.0);
  assert.equal(idleCutoff, 180.0);

  // High speed redline (7000 RPM, 1.0 throttle, 80 km/h)
  const highCutoff = calculateLpfCutoff(7000, 1.0, 80, 'high', 180.0, 450.0);
  assert.equal(highCutoff, 450.0);

  // Boundary clamping
  const clampedCutoff = calculateLpfCutoff(100, -1.0, -50, 'high', 180.0, 450.0);
  assert.ok(clampedCutoff >= 180.0 && clampedCutoff <= 450.0);
  assert.ok(calculateLpfCutoff(9000, 2.0, 200, 'high', 180.0, 450.0) <= 450.0);
});

test('5. Master Volume Attenuation: baseline floor 0.28, ceiling 0.38, 15% shift ducking', () => {
  // Stationary idle: exact baseline floor 0.28
  const idleGain = calculateEngineGain(0, 0, false, 0.28, 0.38);
  assert.equal(idleGain, 0.28);

  // Full throttle redline: capped at 0.38 max ceiling
  const fullGain = calculateEngineGain(1.0, 1.0, false, 0.28, 0.38);
  assert.equal(fullGain, 0.38);

  // During gear shift: ducks by 15% (gain * 0.85)
  const shiftingGain = calculateEngineGain(1.0, 1.0, true, 0.28, 0.38);
  assert.equal(Math.round((shiftingGain / fullGain) * 100) / 100, 0.85);

  // Idle shifting gain
  const idleShiftingGain = calculateEngineGain(0, 0, true, 0.28, 0.38);
  assert.equal(Math.round((idleShiftingGain / idleGain) * 100) / 100, 0.85);
});

test('6. Shift Mechanical Clunk: 55 Hz -> 15 Hz, 0.07s burst, max volume 0.30', () => {
  const clunk = calculateShiftClunkParams();
  assert.equal(clunk.durationSec, 0.07);
  assert.equal(clunk.startFreqHz, 55.0);
  assert.equal(clunk.endFreqHz, 15.0);
  assert.equal(clunk.maxVolume, 0.30);
});

test('7. LOW Range Gear-Mesh Whine: 120 Hz to 220 Hz bandpass noise', () => {
  // Silent when stationary
  const stationaryLow = calculateLowRangeGearMesh(0, 'low');
  assert.equal(stationaryLow.active, false);
  assert.equal(stationaryLow.volume, 0);

  // Silent in HIGH range even at highway speed
  const highwayHigh = calculateLowRangeGearMesh(70, 'high');
  assert.equal(highwayHigh.active, false);
  assert.equal(highwayHigh.volume, 0);

  // Active in LOW range crawl (~5 km/h)
  const crawlLow = calculateLowRangeGearMesh(5.0, 'low');
  assert.equal(crawlLow.active, true);
  assert.ok(crawlLow.centerFreqHz >= 120 && crawlLow.centerFreqHz <= 140);
  assert.ok(crawlLow.volume > 0);

  // Active in LOW range top speed (45 km/h)
  const topLow = calculateLowRangeGearMesh(45, 'low');
  assert.equal(topLow.active, true);
  assert.equal(Math.round(topLow.centerFreqHz), 220);
  assert.ok(topLow.volume <= 0.045);
});
