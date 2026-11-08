/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import test from 'node:test';
import assert from 'node:assert/strict';
import {
  VehicleAudio,
  DEFAULT_AUDIO_CONFIG,
  calculatePlaybackRate,
  calculateLpfCutoff,
  calculateEngineGain,
  calculateLowRangeGearMesh,
  calculateShiftClunkParams,
} from './audio';

test('1. VehicleAudio instantiates with silent default configuration', () => {
  const audio = new VehicleAudio();
  assert.equal(audio.config.masterVolume, 0.0);
  assert.equal(audio.config.idleGainFloor, 0.0);
  assert.equal(audio.config.sampleUrl, '');
  assert.equal(audio.isSampleLoaded(), false);
  assert.equal(audio.getPlaybackRate(), 0.0);
  assert.equal(audio.getFilterCutoff(), 0.0);
});

test('2. Safe API Stubs: init(), unlock(), and dispose() execute cleanly without errors', () => {
  const audio = new VehicleAudio();
  // Safe execution in headless/browser environments
  audio.init();
  audio.unlock();
  assert.equal(audio.isAudioUnlocked(), true);

  audio.dispose();
  assert.equal(audio.getContextState(), 'uninitialized');
});

test('3. Test Scenario A & B: update() is completely silent across all driving states with zero errors', () => {
  const audio = new VehicleAudio();
  audio.init();
  audio.unlock();

  // A. Idle state
  assert.doesNotThrow(() => {
    audio.update(550, 0, 0.016, false, 0, 'high');
  }, 'Idle produces zero errors');

  // B. Full throttle acceleration & redline
  assert.doesNotThrow(() => {
    audio.update(7000, 1.0, 0.016, false, 80, 'high');
  }, 'Acceleration produces zero errors');

  // C. Gear shifting
  assert.doesNotThrow(() => {
    audio.update(4500, 0.5, 0.016, true, 40, 'high');
  }, 'Shifting gears produces zero errors');

  // D. Reverse crawling
  assert.doesNotThrow(() => {
    audio.update(900, -1.0, 0.016, false, -8, 'high');
  }, 'Reverse produces zero errors');

  // E. LOW range crawling
  assert.doesNotThrow(() => {
    audio.update(1200, 0.3, 0.016, false, 6, 'low');
  }, 'LOW range produces zero errors');

  // F. Object-parameter variant
  assert.doesNotThrow(() => {
    audio.update({
      engineRpm: 3200,
      throttle: 0.8,
      dt: 0.016,
      isShifting: false,
      speedKmh: 45,
      rangeMode: 'high',
    });
  }, 'Object-param variant produces zero errors');
});

test('4. Safe No-Op Shift Clunk: playShiftClunk and triggerShiftClunk execute cleanly without sound', () => {
  const audio = new VehicleAudio();
  audio.init();
  audio.unlock();

  assert.doesNotThrow(() => {
    audio.playShiftClunk(0.65);
    audio.triggerShiftClunk(0.30);
  });
});

test('5. Backward compatibility helper stubs return zero/silent values', () => {
  assert.equal(calculatePlaybackRate(), 0.0);
  assert.equal(calculateLpfCutoff(), 0.0);
  assert.equal(calculateEngineGain(), 0.0);
  assert.deepEqual(calculateLowRangeGearMesh(), { active: false, centerFreqHz: 0.0, volume: 0.0 });
  assert.deepEqual(calculateShiftClunkParams(), { durationSec: 0.0, startFreqHz: 0.0, endFreqHz: 0.0, maxVolume: 0.0 });
});
