import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { RigidBody } from './rigidBody';
import { SuspensionSystem } from './suspension';
import { WheelDynamics } from './wheel';
import { BrakeSystem } from './brakes';
import { TestGround } from '../world/testGround';

function setupSlopeVehicle() {
  const body = new RigidBody();
  const susp = new SuspensionSystem();
  const wheelsDyn = [
    new WheelDynamics('FL'),
    new WheelDynamics('FR'),
    new WheelDynamics('RL'),
    new WheelDynamics('RR'),
  ];
  const brakes = new BrakeSystem();
  const ground = new TestGround();
  const dt = 1 / 120;

  // Position on the 20° slope (z = 425 on Zone 3 parking hill)
  const groundQuery = ground.getHeightNormalSurface(0, 425);
  const bodyForward = new THREE.Vector3(0, 0, 1);
  const dot = bodyForward.dot(groundQuery.normal);
  const groundForward = bodyForward.clone().addScaledVector(groundQuery.normal, -dot).normalize();
  const q = new THREE.Quaternion().setFromUnitVectors(bodyForward, groundForward);

  body.position.set(0, groundQuery.height + 1.06, 425);
  body.quaternion.copy(q);

  // Settle suspension onto ground for 180 steps (1.5 seconds) with handbrake
  for (let s = 0; s < 180; s++) {
    susp.update(body, ground, dt);
    brakes.update(0.0, true, wheelsDyn, body.linearVelocity.length(), dt);
    for (let i = 0; i < 4; i++) {
      const qS = ground.getHeightNormalSurface(susp.wheels[i].contactPoint.x, susp.wheels[i].contactPoint.z);
      wheelsDyn[i].update(susp.wheels[i], body, 0, brakes.wheelBrakeTorques[i], qS.surface, dt, i >= 2);
    }
    body.integrate(dt);
  }

  return { body, susp, wheelsDyn, brakes, ground, dt };
}

test('Stage A6: 20-Degree Hill Hold — Handbrake locks vehicle with zero drift over 30 seconds', () => {
  const { body, susp, wheelsDyn, brakes, ground, dt } = setupSlopeVehicle();

  // Ensure rear wheels are stuck
  assert.equal(wheelsDyn[2].isStuck, true, 'Rear Left wheel is in STICK state');
  assert.equal(wheelsDyn[3].isStuck, true, 'Rear Right wheel is in STICK state');

  const initialZ = body.position.z;
  const initialY = body.position.y;

  // Simulate 30 full seconds (3600 steps at 120 Hz) with throttle = 0, handbrake active, foot brake released
  for (let s = 0; s < 3600; s++) {
    susp.update(body, ground, dt);
    brakes.update(0.0, true, wheelsDyn, body.linearVelocity.length(), dt);
    for (let i = 0; i < 4; i++) {
      const q = ground.getHeightNormalSurface(susp.wheels[i].contactPoint.x, susp.wheels[i].contactPoint.z);
      wheelsDyn[i].update(susp.wheels[i], body, 0, brakes.wheelBrakeTorques[i], q.surface, dt, i >= 2);
    }
    body.integrate(dt);
  }

  const speedKmh = body.linearVelocity.length() * 3.6;
  const driftZ = Math.abs(body.position.z - initialZ);
  const driftY = Math.abs(body.position.y - initialY);

  assert.ok(speedKmh < 0.01, `Speed must be 0.00 km/h, got ${speedKmh.toFixed(4)} km/h`);
  assert.ok(driftZ < 0.05, `Longitudinal drift must be < 50mm over 30s, got ${(driftZ * 1000).toFixed(2)} mm`);
  assert.ok(driftY < 0.05, `Vertical drift must be < 50mm over 30s, got ${(driftY * 1000).toFixed(2)} mm`);
  assert.equal(wheelsDyn[2].omega, 0, 'RL wheel angular velocity is locked to 0');
  assert.equal(wheelsDyn[3].omega, 0, 'RR wheel angular velocity is locked to 0');
});

test('Stage A6: Throttle Resistance — Light/moderate throttle stays locked, high torque breaks away', () => {
  const { body, susp, wheelsDyn, brakes, ground, dt } = setupSlopeVehicle();

  assert.equal(wheelsDyn[2].isStuck, true, 'Initially stuck');

  // Apply moderate drive torque (400 N*m per rear wheel) against handbrake for 1 second (120 steps)
  for (let s = 0; s < 120; s++) {
    susp.update(body, ground, dt);
    brakes.update(0.0, true, wheelsDyn, body.linearVelocity.length(), dt);
    for (let i = 0; i < 4; i++) {
      const q = ground.getHeightNormalSurface(susp.wheels[i].contactPoint.x, susp.wheels[i].contactPoint.z);
      const driveT = i >= 2 ? 400 : 0;
      wheelsDyn[i].update(susp.wheels[i], body, driveT, brakes.wheelBrakeTorques[i], q.surface, dt, i >= 2);
    }
    body.integrate(dt);
  }

  assert.equal(wheelsDyn[2].isStuck, true, 'Remains stuck under moderate throttle');
  assert.ok(body.linearVelocity.length() * 3.6 < 0.1, 'Vehicle remains stationary under moderate throttle');

  // Now apply high engine torque exceeding static traction limit (3500 N*m per rear wheel)
  for (let s = 0; s < 30; s++) {
    susp.update(body, ground, dt);
    brakes.update(0.0, true, wheelsDyn, body.linearVelocity.length(), dt);
    for (let i = 0; i < 4; i++) {
      const q = ground.getHeightNormalSurface(susp.wheels[i].contactPoint.x, susp.wheels[i].contactPoint.z);
      const driveT = i >= 2 ? 3500 : 0;
      wheelsDyn[i].update(susp.wheels[i], body, driveT, brakes.wheelBrakeTorques[i], q.surface, dt, i >= 2);
    }
    body.integrate(dt);
  }

  assert.equal(wheelsDyn[2].isStuck, false, 'Wheels cleanly break away under massive drive torque');
});

test('Stage A6: Smooth Release — Disengaging handbrake smoothly restores free rolling without force pops', () => {
  const { body, susp, wheelsDyn, brakes, ground, dt } = setupSlopeVehicle();

  assert.equal(wheelsDyn[2].isStuck, true, 'Rear wheel is stuck');

  // Now release handbrake
  susp.update(body, ground, dt);
  brakes.update(0.0, false, wheelsDyn, 0, dt);
  for (let i = 0; i < 4; i++) {
    const q = ground.getHeightNormalSurface(susp.wheels[i].contactPoint.x, susp.wheels[i].contactPoint.z);
    wheelsDyn[i].update(susp.wheels[i], body, 0, brakes.wheelBrakeTorques[i], q.surface, dt, false);
  }

  assert.equal(wheelsDyn[2].isStuck, false, 'Instant transition out of STICK state');
  // Check no crazy forces or NaNs
  assert.ok(Number.isFinite(wheelsDyn[2].Fx), 'Fx is finite');
  assert.ok(Math.abs(wheelsDyn[2].Fx) < 10000, `No force spikes on release, got ${wheelsDyn[2].Fx} N`);
});
