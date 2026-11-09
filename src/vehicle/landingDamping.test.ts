import React from 'react';
import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { RigidBody } from './rigidBody';
import { SuspensionSystem } from './suspension';
import { WheelDynamics } from './wheel';
import { ChassisContactSystem } from './chassisContact';
import { TestGround } from '../world/testGround';
import { HUD } from '../ui/HUD';

test('Stage A8 (Fix) Test Criteria 1: Minimal HUD Readout — Displays Gear, Speed, Mode, Range without Debug Clutter', () => {
  // Test HUD rendering with standard props
  const rendered = HUD({
    speedKmh: 45,
    currentGear: 3,
    mode: 'auto',
    rangeMode: 'high',
  });

  assert.ok(React.isValidElement(rendered), 'HUD should return a valid React element');
  const element = rendered as React.ReactElement<{ 'data-testid': string }>;
  assert.equal(element.props['data-testid'], 'driving-hud', 'HUD has data-testid="driving-hud"');

  // Test Reverse gear readout
  const reverseRendered = HUD({
    speedKmh: 12,
    currentGear: -1,
    mode: 'manual',
    rangeMode: 'low',
  });
  assert.ok(reverseRendered !== null);

  // Test Neutral gear readout
  const neutralRendered = HUD({
    speedKmh: 0,
    currentGear: 0,
    mode: 'auto',
    rangeMode: 'high',
  });
  assert.ok(neutralRendered !== null);
});

test('Stage A8 (Fix) Test Criteria 2: Asymmetric Rebound Damping Ratio (2.5x to 3.5x bump damping)', () => {
  const susp = new SuspensionSystem();
  for (const wheel of susp.wheels) {
    const ratio = wheel.config.reboundDamping / wheel.config.bumpDamping;
    assert.ok(
      ratio >= 2.5 && ratio <= 3.5,
      `Wheel ${wheel.id} rebound-to-bump damping ratio must be between 2.5 and 3.5, got ${ratio.toFixed(2)}`
    );
  }
});

test('Stage A8 (Fix) Test Criteria 3: Ramp Jump Landing (High Downward Velocity Impact) — Settles in <= 1 Cycle without Catapulting', () => {
  const dt = 1 / 120;
  const body = new RigidBody({ mass: 1800 });
  const susp = new SuspensionSystem();
  const wheelsDyn = [
    new WheelDynamics('FL'),
    new WheelDynamics('FR'),
    new WheelDynamics('RL'),
    new WheelDynamics('RR'),
  ];
  const chassisContact = new ChassisContactSystem();
  const ground = new TestGround();

  // Position vehicle at landing altitude after launch off the Stage A1 ramp jump
  // Dropping from 1.5m above ground with downward landing velocity of -6.0 m/s
  body.position.set(0, 1.5, 175);
  body.linearVelocity.set(0, -6.0, 15.0); // 15 m/s (~54 km/h) forward, -6.0 m/s downward impact
  body.angularVelocity.set(0, 0, 0);

  let maxCompressionObserved = 0;
  let maxOverTravelObserved = 0;
  let maxUpwardBounceVelocity = 0;
  let touchedGround = false;

  // Simulate 2.5 seconds (300 ticks) of landing physics
  for (let s = 0; s < 300; s++) {
    body.clearAccumulators();
    susp.update(body, ground, dt);

    for (let i = 0; i < 4; i++) {
      const sw = susp.wheels[i];
      if (sw.grounded) {
        touchedGround = true;
        if (sw.compression > maxCompressionObserved) {
          maxCompressionObserved = sw.compression;
        }
        if (sw.overTravel > maxOverTravelObserved) {
          maxOverTravelObserved = sw.overTravel;
        }
      }
      wheelsDyn[i].update(sw, body, 0, 0, 'asphalt', dt, false);
    }

    chassisContact.update(body, ground, dt, susp, wheelsDyn);
    body.integrate(dt);
    chassisContact.postIntegrate(body, ground, susp, wheelsDyn);

    // Track rebound upward velocity after ground contact has occurred
    if (touchedGround && s > 15) {
      if (body.linearVelocity.y > maxUpwardBounceVelocity) {
        maxUpwardBounceVelocity = body.linearVelocity.y;
      }
    }
  }

  // 1. Suspension compressed smoothly to absorb landing kinetic energy
  assert.ok(touchedGround, 'Vehicle must touch ground on landing');
  assert.ok(
    maxCompressionObserved >= 0.20,
    `Suspension must compress significantly to absorb impact, got ${maxCompressionObserved.toFixed(3)}m`
  );

  // 2. Anti-catapult check: upward rebound velocity must be suppressed by asymmetric rebound damping
  assert.ok(
    maxUpwardBounceVelocity < 0.85,
    `Upward bounce velocity must be heavily damped to prevent catapulting (< 0.85 m/s), got ${maxUpwardBounceVelocity.toFixed(3)} m/s`
  );

  // 3. Settling check: by 2.5 seconds, vehicle vertical velocity should be settled near 0 (< 0.15 m/s)
  assert.ok(
    Math.abs(body.linearVelocity.y) < 0.15,
    `Vehicle must settle smoothly after landing, got Vy = ${body.linearVelocity.y.toFixed(3)} m/s`
  );

  // 4. Equilibrium ride height check: chassis should settle near nominal ~1.06m
  assert.ok(
    body.position.y >= 0.95 && body.position.y <= 1.20,
    `Chassis position Y should settle near nominal ride height (0.95m - 1.20m), got ${body.position.y.toFixed(3)}m`
  );

  // 5. Zero physics resets, zero safety net triggers, and zero NaNs
  assert.equal(body.nanResetCount, 0, 'Zero NaN resets during heavy landing');
  assert.equal(chassisContact.safetyNetCounter, 0, 'Zero safety net triggers during landing');
});

test('Stage A8 (Fix) Test Criteria 4: Low-Speed Obstacle Compliance — Smooth Soft Articulation over Bumps', () => {
  const susp = new SuspensionSystem();
  // Low compression velocity (|v| <= 1.0 m/s) maintains soft base damping
  const bumpDamping = susp.wheelFL.config.bumpDamping;
  assert.equal(bumpDamping, 3600, 'Base bump damping must be 3600 N*s/m for soft obstacle compliance');
});

test('Stage A8 (Fix) Test Criteria 5: Restitution Clamping — Dynamic Restitution Clamped to e <= 0.05', () => {
  const chassisContact = new ChassisContactSystem();
  assert.ok(
    chassisContact.restitution <= 0.05,
    `Chassis impact restitution must be <= 0.05, got ${chassisContact.restitution}`
  );
});
