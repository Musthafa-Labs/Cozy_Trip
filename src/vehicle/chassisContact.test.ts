import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { RigidBody } from './rigidBody';
import { ChassisContactSystem } from './chassisContact';
import { TestGround } from '../world/testGround';

test('Chassis Contact: Vehicle resting upright on wheels does not penetrate terrain', () => {
  const body = new RigidBody();
  const contact = new ChassisContactSystem();
  const ground = new TestGround();

  body.position.set(0, 0.7, 0);
  for (let s = 0; s < 120; s++) {
    contact.update(body, ground, 1 / 120);
    body.integrate(1 / 120);
    contact.postIntegrate(body, ground);
  }

  assert.ok(Number.isFinite(body.position.y), 'Position Y is finite');
  assert.ok(body.position.y > 0.05, `Position Y should remain above ground, got ${body.position.y}`);
  assert.equal(body.nanResetCount, 0, 'Zero NaN resets');
});

test('Chassis Contact: Vehicle resting on its side settles naturally without launching', () => {
  const body = new RigidBody();
  const contact = new ChassisContactSystem();
  const ground = new TestGround();

  body.position.set(0, 1.0, 0);
  body.quaternion.setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI / 2);
  body.linearVelocity.set(0, 0, 0);
  body.angularVelocity.set(0, 0, 0);

  for (let s = 0; s < 360; s++) {
    contact.update(body, ground, 1 / 120);
    body.integrate(1 / 120);
    contact.postIntegrate(body, ground);
  }

  assert.ok(Number.isFinite(body.position.y), 'Position Y is finite');
  assert.ok(Math.abs(body.linearVelocity.y) < 0.1, `Vertical velocity settled near zero, got ${body.linearVelocity.y}`);
  assert.ok(body.angularVelocity.length() < 0.2, `Angular velocity settled near zero, got ${body.angularVelocity.length()}`);
  assert.ok(contact.maxPenetrationMeters < 0.15, `Penetration bounded, got ${contact.maxPenetrationMeters}`);
  assert.equal(body.nanResetCount, 0, 'Zero NaN resets');
});

test('Chassis Contact: Vehicle resting completely upside down on roof settles stably', () => {
  const body = new RigidBody();
  const contact = new ChassisContactSystem();
  const ground = new TestGround();

  body.position.set(0, 1.45, 0);
  body.quaternion.setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI);
  body.linearVelocity.set(0, 0, 0);
  body.angularVelocity.set(0, 0, 0);

  for (let s = 0; s < 240; s++) {
    contact.update(body, ground, 1 / 120);
    body.integrate(1 / 120);
    contact.postIntegrate(body, ground);
  }

  assert.ok(Number.isFinite(body.position.y), 'Position Y is finite');
  assert.ok(body.position.y > 1.35 && body.position.y < 1.50, `Resting roof height realistic, got ${body.position.y}`);
  assert.ok(Math.abs(body.linearVelocity.y) < 0.1, `Vertical velocity settled near zero, got ${body.linearVelocity.y}`);
  assert.ok(body.angularVelocity.length() < 0.2, `Angular velocity settled near zero, got ${body.angularVelocity.length()}`);
  assert.equal(body.nanResetCount, 0, 'Zero NaN resets');
});

test('Chassis Contact: Landing on roof after a flip dissipates kinetic energy without continuous launching or spinning', () => {
  const body = new RigidBody();
  const contact = new ChassisContactSystem();
  const ground = new TestGround();

  // Falling from 2.5m upside-down with downward speed and roll/pitch spin
  body.position.set(0, 2.5, 0);
  body.quaternion.setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI * 0.95);
  body.linearVelocity.set(1.0, -4.0, 0.5);
  body.angularVelocity.set(1.0, 0.2, 2.0);

  let maxReboundSpeed = 0;
  for (let s = 0; s < 480; s++) {
    contact.update(body, ground, 1 / 120);
    body.integrate(1 / 120);
    contact.postIntegrate(body, ground);

    // After initial contact at step ~30, measure any secondary launch speeds
    if (s > 60 && body.linearVelocity.y > maxReboundSpeed) {
      maxReboundSpeed = body.linearVelocity.y;
    }
  }

  assert.ok(maxReboundSpeed < 3.0, `Vehicle never violently launches into the air, max rebound vy: ${maxReboundSpeed}`);
  assert.ok(body.position.y < 2.0, `Vehicle stays on the ground, got y=${body.position.y}`);
  assert.ok(Math.abs(body.linearVelocity.y) < 0.2, `Final vertical velocity settled, got ${body.linearVelocity.y}`);
  assert.ok(body.angularVelocity.length() < 0.5, `Final angular velocity settled without spinning out, got ${body.angularVelocity.length()}`);
  assert.equal(body.nanResetCount, 0, 'Zero NaN resets');
});

test('Chassis Contact: Safety-net test pose (K) recovers gracefully without rocket launch', () => {
  const body = new RigidBody();
  const contact = new ChassisContactSystem();
  const ground = new TestGround();

  // K test pose: y = -0.45m upright
  body.position.set(0, -0.45, 0);
  body.quaternion.identity();
  body.linearVelocity.set(0, 0, 0);
  body.angularVelocity.set(0, 0, 0);

  for (let s = 0; s < 240; s++) {
    contact.update(body, ground, 1 / 120);
    body.integrate(1 / 120);
    contact.postIntegrate(body, ground);
  }

  assert.ok(contact.safetyNetCounter > 0, 'Safety net triggered successfully');
  assert.ok(body.position.y > 0.05 && body.position.y < 0.85, `Position gracefully recovered above ground, got y=${body.position.y}`);
  assert.ok(Math.abs(body.linearVelocity.y) < 0.2, `Velocity settled near zero without flying to space, got vy=${body.linearVelocity.y}`);
  assert.ok(body.angularVelocity.length() < 0.2, `Angular velocity settled near zero, got w=${body.angularVelocity.length()}`);
  assert.equal(body.nanResetCount, 0, 'Zero NaN resets');
});
