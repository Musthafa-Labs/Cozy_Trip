/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { RigidBody } from './rigidBody';
import { SuspensionSystem } from './suspension';
import { WheelDynamics } from './wheel';
import { BrakeSystem } from './brakes';
import { TestGround } from '../world/testGround';
import { TireModel, SURFACE_PACEJKA } from './tire';

test('Stage A7: Surface Material Property Matrix Verification', () => {
  // Verify all 7 core surfaces have the exact requested parameters
  const asphalt = SURFACE_PACEJKA.asphalt;
  assert.equal(asphalt.muStatic, 0.95);
  assert.equal(asphalt.muKinetic, 0.85);
  assert.equal(asphalt.rollingResCoeff, 0.015);

  const dirt = SURFACE_PACEJKA.dirt;
  assert.equal(dirt.muStatic, 0.75);
  assert.equal(dirt.muKinetic, 0.65);
  assert.equal(dirt.rollingResCoeff, 0.035);

  const gravel = SURFACE_PACEJKA.gravel;
  assert.equal(gravel.muStatic, 0.55);
  assert.equal(gravel.muKinetic, 0.45);
  assert.equal(gravel.rollingResCoeff, 0.060);

  const mud = SURFACE_PACEJKA.mud;
  assert.equal(mud.muStatic, 0.38);
  assert.equal(mud.muKinetic, 0.28);
  assert.equal(mud.rollingResCoeff, 0.120);

  const rock = SURFACE_PACEJKA.rock;
  assert.equal(rock.muStatic, 0.88);
  assert.equal(rock.muKinetic, 0.78);
  assert.equal(rock.rollingResCoeff, 0.025);

  const ice = SURFACE_PACEJKA.ice;
  assert.equal(ice.muStatic, 0.22);
  assert.equal(ice.muKinetic, 0.15);
  assert.equal(ice.rollingResCoeff, 0.040);

  const grass = SURFACE_PACEJKA.grass;
  assert.equal(grass.muStatic, 0.42);
  assert.equal(grass.muKinetic, 0.32);
  assert.equal(grass.rollingResCoeff, 0.050);
});

test('Stage A7: Terrain Queries return correct surface IDs across Zone 9 strips', () => {
  const ground = new TestGround();

  // Test strip centers along Z for X = 0 (on road)
  assert.equal(ground.getHeightNormalSurface(0, 800).surface, 'asphalt');
  assert.equal(ground.getHeightNormalSurface(0, 820).surface, 'dirt');
  assert.equal(ground.getHeightNormalSurface(0, 840).surface, 'gravel');
  assert.equal(ground.getHeightNormalSurface(0, 860).surface, 'mud');
  assert.equal(ground.getHeightNormalSurface(0, 880).surface, 'rock');
  assert.equal(ground.getHeightNormalSurface(0, 900).surface, 'ice');
  assert.equal(ground.getHeightNormalSurface(0, 920).surface, 'grass');
  assert.equal(ground.getHeightNormalSurface(0, 940).surface, 'sand');

  // Off-road shoulder query returns dirt
  assert.equal(ground.getHeightNormalSurface(12, 860).surface, 'dirt');

  // Surface Pack test station exists at (0, 1.06, 780)
  const station = ground.stations.find((s) => s.id === 'surface_pack_test');
  assert.ok(station, 'surface_pack_test station must be registered');
  assert.equal(station.position.z, 780);
  assert.equal(station.headingRad, 0);
});

test('Stage A7 Test Criteria 1: Friction Scaling — Snow/Ice vs Asphalt Wheelspin & Braking', () => {
  const Fz = 4415; // Nominal load

  // Test Tire evaluation on Asphalt vs Ice
  const asphaltResult = TireModel.evaluate(0.20, 0, Fz, 'asphalt');
  const iceResult = TireModel.evaluate(0.20, 0, Fz, 'ice');

  // Ice longitudinal tractive force must be drastically lower than Asphalt (>3.5x difference)
  assert.ok(asphaltResult.Fx > 3500, `Asphalt tractive force should be high, got ${asphaltResult.Fx}`);
  assert.ok(iceResult.Fx < 1000, `Ice tractive force should be low, got ${iceResult.Fx}`);
  const ratio = asphaltResult.Fx / iceResult.Fx;
  assert.ok(ratio > 3.5, `Asphalt to Ice grip ratio must exceed 3.5, got ${ratio}`);
});

test('Stage A7 Test Criteria 2: Rolling Resistance — Neutral Coastdown at 30 km/h (Mud vs Asphalt)', () => {
  const dt = 1 / 120;
  const initialSpeed = 30 / 3.6; // 8.33 m/s
  const ground = new TestGround();

  const simulateCoastdown = (startZ: number) => {
    const body = new RigidBody({ mass: 1800 });
    body.position.set(0, 1.06, startZ);
    body.linearVelocity.set(0, 0, initialSpeed);

    const susp = new SuspensionSystem();
    const wheels = [
      new WheelDynamics('FL'),
      new WheelDynamics('FR'),
      new WheelDynamics('RL'),
      new WheelDynamics('RR'),
    ];

    for (const w of wheels) {
      w.omega = initialSpeed / w.radius;
    }

    for (let step = 0; step < 60; step++) {
      susp.update(body, ground, dt);

      for (let i = 0; i < 4; i++) {
        const qS = ground.getHeightNormalSurface(susp.wheels[i].contactPoint.x, susp.wheels[i].contactPoint.z);
        wheels[i].update(susp.wheels[i], body, 0, 0, qS.surface, dt, false);
      }
      body.integrate(dt);
    }

    return body.linearVelocity.z;
  };

  // Start in Asphalt strip (Z = 795) vs Mud strip (Z = 855)
  const finalSpeedAsphalt = simulateCoastdown(795);
  const finalSpeedMud = simulateCoastdown(855);

  const deltaVAsphalt = initialSpeed - finalSpeedAsphalt;
  const deltaVMud = initialSpeed - finalSpeedMud;

  assert.ok(deltaVMud > deltaVAsphalt * 3.0, `Mud deceleration (${deltaVMud.toFixed(3)} m/s) must be > 3x asphalt (${deltaVAsphalt.toFixed(3)} m/s)`);
});

test('Stage A7 Test Criteria 3: Lateral Slip on Wet Grass — Cornering slip vs Asphalt', () => {
  const Fz = 4415;
  const slipAngleRad = 10 * (Math.PI / 180); // 10 degrees steer slip

  const grassTire = TireModel.evaluate(0, slipAngleRad, Fz, 'grass');
  const asphaltTire = TireModel.evaluate(0, slipAngleRad, Fz, 'asphalt');

  assert.ok(Math.abs(grassTire.Fy) < Math.abs(asphaltTire.Fy) * 0.55,
    `Grass lateral force (${Math.abs(grassTire.Fy)}) should be under 55% of asphalt (${Math.abs(asphaltTire.Fy)})`
  );
});

test('Stage A7 Test Criteria 4: No Regressions on Handbrake Stiction and Slope Hold', () => {
  const dt = 1 / 120;
  const ground = new TestGround();
  const body = new RigidBody();
  const susp = new SuspensionSystem();
  const wheelsDyn = [
    new WheelDynamics('FL'),
    new WheelDynamics('FR'),
    new WheelDynamics('RL'),
    new WheelDynamics('RR'),
  ];
  const brakes = new BrakeSystem();

  // Position at parking hill incline (Z = 425)
  const groundQuery = ground.getHeightNormalSurface(0, 425);
  const bodyForward = new THREE.Vector3(0, 0, 1);
  const dot = bodyForward.dot(groundQuery.normal);
  const groundForward = bodyForward.clone().addScaledVector(groundQuery.normal, -dot).normalize();
  const q = new THREE.Quaternion().setFromUnitVectors(bodyForward, groundForward);

  body.position.set(0, groundQuery.height + 1.06, 425);
  body.quaternion.copy(q);

  // Settle vehicle on slope with handbrake engaged for 240 steps (2 seconds)
  for (let s = 0; s < 240; s++) {
    susp.update(body, ground, dt);
    brakes.update(0.0, true, wheelsDyn, body.linearVelocity.length(), dt);
    for (let i = 0; i < 4; i++) {
      const qS = ground.getHeightNormalSurface(susp.wheels[i].contactPoint.x, susp.wheels[i].contactPoint.z);
      wheelsDyn[i].update(susp.wheels[i], body, 0, brakes.wheelBrakeTorques[i], qS.surface, dt, i >= 2);
    }
    body.integrate(dt);
  }

  // Ensure rear wheels locked in STICK state
  assert.equal(wheelsDyn[2].isStuck, true, 'RL wheel must be in STICK state');
  assert.equal(wheelsDyn[3].isStuck, true, 'RR wheel must be in STICK state');

  // Measure drift over 240 steps (2 seconds)
  const zStart = body.position.z;
  for (let s = 0; s < 240; s++) {
    susp.update(body, ground, dt);
    brakes.update(0.0, true, wheelsDyn, body.linearVelocity.length(), dt);
    for (let i = 0; i < 4; i++) {
      const qS = ground.getHeightNormalSurface(susp.wheels[i].contactPoint.x, susp.wheels[i].contactPoint.z);
      wheelsDyn[i].update(susp.wheels[i], body, 0, brakes.wheelBrakeTorques[i], qS.surface, dt, i >= 2);
    }
    body.integrate(dt);
  }
  const zEnd = body.position.z;
  const drift = Math.abs(zEnd - zStart);
  const speedKmh = body.linearVelocity.length() * 3.6;

  assert.ok(drift < 0.05, `Downhill drift must be < 50mm, got ${(drift * 1000).toFixed(2)}mm`);
  assert.ok(speedKmh < 0.05, `Speed must be stationary, got ${speedKmh.toFixed(4)} km/h`);
  assert.equal(wheelsDyn[2].isStuck, true, 'Rear wheel 2 remains stuck');
  assert.equal(wheelsDyn[3].isStuck, true, 'Rear wheel 3 remains stuck');
});
