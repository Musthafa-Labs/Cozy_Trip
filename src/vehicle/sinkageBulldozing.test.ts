/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { RigidBody } from './rigidBody';
import { SuspensionSystem, WheelState } from './suspension';
import { WheelDynamics } from './wheel';
import { TestGround } from '../world/testGround';
import { DebugOverlay } from '../ui/DebugOverlay';

test('Stage A8 Test Criteria 1: Clean UI — DebugOverlay renders null', () => {
  // Test that DebugOverlay returns null and exports no debug text
  const rendered = DebugOverlay({});
  assert.equal(rendered, null, 'DebugOverlay must return null for a clean UI');
});

test('Stage A8 Test Criteria 2: Visible & Physical Sinkage — Mud (0.08-0.12m) and Sand (0.06-0.08m)', () => {
  const dt = 1 / 120;
  const body = new RigidBody({ mass: 1800 });
  body.position.set(0, 1.06, 0);

  const susp = new SuspensionSystem();
  const wheelMud = new WheelDynamics('FL');
  const wheelSand = new WheelDynamics('FR');

  // Nominal grounded contact state
  const swMud = susp.wheels[0];
  swMud.grounded = true;
  swMud.normalLoad = 4415; // 1800kg / 4 * 9.81
  swMud.contactPoint.set(0.8, 0, 1.3);
  swMud.wheelWorldPosition.set(0.8, 0.38, 1.3);

  const initialHubY = swMud.wheelWorldPosition.y;

  // Simulate standing in Wet Mud for 30 steps
  for (let s = 0; s < 30; s++) {
    swMud.wheelWorldPosition.set(0.8, 0.38, 1.3);
    wheelMud.update(swMud, body, 0, 0, 'mud', dt, false);
  }

  // Sinkage depth must be between 0.08m and 0.12m
  assert.ok(
    wheelMud.sinkageDepth >= 0.08 && wheelMud.sinkageDepth <= 0.12,
    `Mud sinkage depth must be between 0.08m and 0.12m, got ${wheelMud.sinkageDepth.toFixed(4)}m`
  );

  // Wheel hub position must be physically and visibly lowered by sinkage depth
  const hubDrop = initialHubY - swMud.wheelWorldPosition.y;
  assert.ok(
    hubDrop >= 0.08 && hubDrop <= 0.125,
    `Wheel hub must be lowered by sinkage depth, got drop of ${hubDrop.toFixed(4)}m`
  );

  // Simulate standing in Sand for 30 steps
  const swSand = susp.wheels[1];
  swSand.grounded = true;
  swSand.normalLoad = 4415;
  swSand.contactPoint.set(-0.8, 0, 1.3);

  for (let s = 0; s < 30; s++) {
    swSand.wheelWorldPosition.set(-0.8, 0.38, 1.3);
    wheelSand.update(swSand, body, 0, 0, 'sand', dt, false);
  }

  // Sand sinkage depth must be between 0.06m and 0.08m
  assert.ok(
    wheelSand.sinkageDepth >= 0.06 && wheelSand.sinkageDepth <= 0.08,
    `Sand sinkage depth must be between 0.06m and 0.08m, got ${wheelSand.sinkageDepth.toFixed(4)}m`
  );
});

test('Stage A8 Test Criteria 3: Bulldozing Drag & Dynamic Digging Effect', () => {
  const dt = 1 / 120;
  const body = new RigidBody({ mass: 1800 });
  const susp = new SuspensionSystem();
  const wheel = new WheelDynamics('RL');

  const sw = susp.wheels[2];
  sw.grounded = true;
  sw.normalLoad = 4415;
  sw.contactPoint.set(0.8, 0, -1.3);
  sw.wheelWorldPosition.set(0.8, 0.38, -1.3);

  // 1. Static sinkage at rest in mud
  for (let s = 0; s < 30; s++) {
    wheel.update(sw, body, 0, 0, 'mud', dt, false);
  }
  const staticSinkage = wheel.sinkageDepth;
  const staticBulldozing = wheel.bulldozingForce;
  assert.ok(staticBulldozing > 800, `Bulldozing drag in deep mud should exceed 800N, got ${staticBulldozing.toFixed(1)}N`);

  // 2. High throttle wheelspin digging effect at low speed
  // Apply heavy drive torque causing high wheelspin while vehicle remains near zero speed
  wheel.omega = 40.0; // High wheelspin (burnout in mud)
  for (let s = 0; s < 30; s++) {
    wheel.update(sw, body, 800, 0, 'mud', dt, false);
  }

  // Excessive wheelspin must dig the wheel deeper toward max sinkage (0.12m)
  assert.ok(
    wheel.sinkageDepth > staticSinkage,
    `Wheelspin must dig deeper than static sinkage (${wheel.sinkageDepth.toFixed(4)} > ${staticSinkage.toFixed(4)})`
  );
  assert.ok(
    wheel.sinkageDepth >= 0.115,
    `Wheelspin should dig down close to max sinkage cap (0.12m), got ${wheel.sinkageDepth.toFixed(4)}m`
  );

  // Bulldozing resistance force must increase quadratically with sinkage depth
  assert.ok(
    wheel.bulldozingForce > staticBulldozing,
    `Bulldozing force after digging (${wheel.bulldozingForce.toFixed(1)}N) must exceed static drag (${staticBulldozing.toFixed(1)}N)`
  );
  assert.ok(
    wheel.bulldozingForce >= 1000,
    `Deep mud bulldozing drag at max sinkage must exceed 1000N per wheel, got ${wheel.bulldozingForce.toFixed(1)}N`
  );
});

test('Stage A8 Test Criteria 4: Zero Hard Ground Deformation (Asphalt, Rock, Ice, Dry Dirt)', () => {
  const dt = 1 / 120;
  const body = new RigidBody({ mass: 1800 });
  const susp = new SuspensionSystem();
  const wheel = new WheelDynamics('FL');

  const sw = susp.wheels[0];
  sw.grounded = true;
  sw.normalLoad = 4415;
  sw.contactPoint.set(0.8, 0, 1.3);
  sw.wheelWorldPosition.set(0.8, 0.38, 1.3);

  const nonDeformableSurfaces = ['asphalt', 'rock', 'ice', 'dirt', 'gravel', 'grass'] as const;

  for (const surface of nonDeformableSurfaces) {
    wheel.reset();
    for (let s = 0; s < 20; s++) {
      wheel.update(sw, body, 200, 0, surface, dt, false);
    }

    assert.equal(
      wheel.sinkageDepth,
      0.0,
      `Surface '${surface}' must maintain strictly 0.00m sinkage depth`
    );
    assert.equal(
      wheel.bulldozingForce,
      0.0,
      `Surface '${surface}' must maintain strictly 0.00N bulldozing drag`
    );
  }
});
