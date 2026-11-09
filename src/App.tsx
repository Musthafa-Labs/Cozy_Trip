/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import React, { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { RigidBody } from './vehicle/rigidBody';
import { SuspensionSystem } from './vehicle/suspension';
import { WheelDynamics } from './vehicle/wheel';
import { SteeringSystem } from './vehicle/steering';
import { Engine } from './vehicle/engine';
import { Drivetrain } from './vehicle/drivetrain';
import { BrakeSystem } from './vehicle/brakes';
import { Aerodynamics } from './vehicle/aero';
import { ChassisContactSystem } from './vehicle/chassisContact';
import { VehicleAudio } from './vehicle/audio';
import { TestGround, TestStation } from './world/testGround';
import { VehicleVisualModel } from './vehicle/vehicleModel';
import { FixedTimestepLoop } from './core/loop';
import { InputManager } from './core/input';
import { HUD } from './ui/HUD';
import { SCRATCH_VEC3_A, clamp, lerp } from './utils/math';

export default function App() {
  const mountRef = useRef<HTMLDivElement>(null);

  // In-game minimal notification message (e.g. transfer case or station shift feedback)
  const [hudMessage, setHudMessage] = useState<string | null>(null);
  const messageTimerRef = useRef<number | null>(null);

  const showHudMessage = (msg: string, durationMs: number = 1500) => {
    if (messageTimerRef.current !== null) {
      clearTimeout(messageTimerRef.current);
    }
    setHudMessage(msg);
    messageTimerRef.current = window.setTimeout(() => {
      setHudMessage(null);
      messageTimerRef.current = null;
    }, durationMs);
  };

  // Minimal driving HUD state: Gear, Mode, Range, Speed (km/h)
  const [hudState, setHudState] = useState({
    speedKmh: 0,
    currentGear: 1,
    mode: 'auto' as 'auto' | 'manual',
    rangeMode: 'high' as 'high' | 'low',
  });
  const lastHudUpdateRef = useRef<number>(0);

  // Live simulation references
  const rigidBodyRef = useRef<RigidBody | null>(null);
  const suspensionRef = useRef<SuspensionSystem | null>(null);
  const wheelsDynRef = useRef<WheelDynamics[]>([]);
  const steeringRef = useRef<SteeringSystem | null>(null);
  const engineRef = useRef<Engine | null>(null);
  const drivetrainRef = useRef<Drivetrain | null>(null);
  const brakesRef = useRef<BrakeSystem | null>(null);
  const aeroRef = useRef<Aerodynamics | null>(null);
  const chassisContactRef = useRef<ChassisContactSystem | null>(null);
  const audioRef = useRef<VehicleAudio | null>(null);
  const groundRef = useRef<TestGround | null>(null);
  const visualModelRef = useRef<VehicleVisualModel | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const controlsRef = useRef<OrbitControls | null>(null);
  const prevCarFollowPos = useRef(new THREE.Vector3(0, 0.85, 0));
  const stationIndexRef = useRef<number>(0);
  const inputRef = useRef<InputManager | null>(null);
  const pendingSidewaysPushRef = useRef<boolean>(false);
  const handbrakeLatchedRef = useRef<boolean>(false);
  const lastGearRef = useRef<number>(1);

  // Preallocated structures for render interpolation
  const prevBodyPos = useRef(new THREE.Vector3());
  const currBodyPos = useRef(new THREE.Vector3());
  const prevBodyQuat = useRef(new THREE.Quaternion());
  const currBodyQuat = useRef(new THREE.Quaternion());

  const prevWheelPos = useRef([new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()]);
  const currWheelPos = useRef([new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()]);

  const prevSteerAngle = useRef([0, 0, 0, 0]);
  const currSteerAngle = useRef([0, 0, 0, 0]);

  const prevSpinAngle = useRef([0, 0, 0, 0]);
  const currSpinAngle = useRef([0, 0, 0, 0]);

  // Interpolated rendering targets
  const renderBodyPos = useRef(new THREE.Vector3());
  const renderBodyQuat = useRef(new THREE.Quaternion());
  const renderWheelPos = useRef([new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()]);
  const renderSteerAngle = useRef([0, 0, 0, 0]);
  const renderSpinAngle = useRef([0, 0, 0, 0]);

  const resetToStation = (index: number) => {
    const ground = groundRef.current;
    const body = rigidBodyRef.current;
    const susp = suspensionRef.current;
    const wheels = wheelsDynRef.current;
    const dt = drivetrainRef.current;
    const brk = brakesRef.current;
    const eng = engineRef.current;
    const str = steeringRef.current;
    const aero = aeroRef.current;
    const ch = chassisContactRef.current;

    if (!ground || !body || !susp) return;

    const stations = ground.stations;
    const idx = ((index % stations.length) + stations.length) % stations.length;
    stationIndexRef.current = idx;
    const station = stations[idx];

    // Reset physics state
    body.position.copy(station.position);
    body.quaternion.setFromAxisAngle(new THREE.Vector3(0, 1, 0), station.headingRad);
    body.linearVelocity.set(0, 0, 0);
    body.angularVelocity.set(0, 0, 0);
    body.clearAccumulators();

    susp.reset();
    wheels.forEach((w) => w.reset());
    dt?.reset();
    brk?.reset();
    eng?.reset();
    if (str) {
      str.currentVirtualAngle = 0;
      str.steerAngleFL = 0;
      str.steerAngleFR = 0;
    }
    aero?.reset();
    ch?.reset();

    prevBodyPos.current.copy(body.position);
    currBodyPos.current.copy(body.position);
    renderBodyPos.current.copy(body.position);

    prevBodyQuat.current.copy(body.quaternion);
    currBodyQuat.current.copy(body.quaternion);
    renderBodyQuat.current.copy(body.quaternion);

    for (let i = 0; i < 4; i++) {
      prevWheelPos.current[i].copy(susp.wheels[i].wheelWorldPosition);
      currWheelPos.current[i].copy(susp.wheels[i].wheelWorldPosition);
      renderWheelPos.current[i].copy(susp.wheels[i].wheelWorldPosition);
      prevSteerAngle.current[i] = 0;
      currSteerAngle.current[i] = 0;
      renderSteerAngle.current[i] = 0;
      prevSpinAngle.current[i] = 0;
      currSpinAngle.current[i] = 0;
      renderSpinAngle.current[i] = 0;
    }

    prevCarFollowPos.current.set(body.position.x, body.position.y + 0.85, body.position.z);

    if (cameraRef.current && controlsRef.current) {
      cameraRef.current.position.set(body.position.x, body.position.y + 2.8, body.position.z - 7.5);
      controlsRef.current.target.set(body.position.x, body.position.y + 0.85, body.position.z);
      controlsRef.current.update();
    }

    showHudMessage(`Station ${idx + 1}/${stations.length}: ${station.name}`);
  };

  const nextStation = () => {
    resetToStation(stationIndexRef.current + 1);
  };

  const applySidewaysPush = () => {
    pendingSidewaysPushRef.current = true;
    showHudMessage('Lateral push impulse applied', 1200);
  };

  const applyRolloverPose = () => {
    const body = rigidBodyRef.current;
    if (!body) return;
    const invQuat = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI);
    body.position.y += 0.8;
    body.quaternion.copy(invQuat);
    body.linearVelocity.set(0, -0.5, 0);
    body.angularVelocity.set(0, 0, 0);
    showHudMessage('Inverted chassis rollover test', 1200);
  };

  const applySafetyNetTestPose = () => {
    const body = rigidBodyRef.current;
    if (!body) return;
    body.position.y = -1.2;
    body.linearVelocity.set(0, -2.0, 0);
    body.angularVelocity.set(0, 0, 0);
    showHudMessage('Chassis ground safety-net recovery', 1200);
  };

  const toggleLayout = () => {
    const dt = drivetrainRef.current;
    if (!dt) return;
    dt.layout = dt.layout === 'AWD' ? 'RWD' : dt.layout === 'RWD' ? 'FWD' : 'AWD';
    showHudMessage(`Drivetrain: ${dt.layout}`, 1200);
  };

  const toggleTransmissionMode = () => {
    const dt = drivetrainRef.current;
    if (!dt) return;
    dt.mode = dt.mode === 'auto' ? 'manual' : 'auto';
    showHudMessage(`Transmission: ${dt.mode.toUpperCase()}`, 1200);
  };

  const toggleAbs = () => {
    const brk = brakesRef.current;
    if (!brk) return;
    brk.config.absEnabled = !brk.config.absEnabled;
    showHudMessage(`ABS: ${brk.config.absEnabled ? 'ON' : 'OFF'}`, 1200);
  };

  const toggleRangeCase = () => {
    const dt = drivetrainRef.current;
    const body = rigidBodyRef.current;
    if (!dt || !body) return;
    const forwardSpeedMs = body.linearVelocity.dot(SCRATCH_VEC3_A.set(0, 0, 1).applyQuaternion(body.quaternion));
    const success = dt.toggleRange(forwardSpeedMs);
    if (success) {
      showHudMessage(`Transfer Case: ${dt.rangeMode.toUpperCase()} RANGE`, 1500);
    } else {
      showHudMessage('Stop vehicle to shift transfer case', 1500);
    }
  };

  const manualShiftUp = () => {
    const dt = drivetrainRef.current;
    const body = rigidBodyRef.current;
    const eng = engineRef.current;
    if (!dt || !body || !eng) return;
    const bodyForward = SCRATCH_VEC3_A.set(0, 0, 1).applyQuaternion(body.quaternion);
    const forwardSpeedMs = body.linearVelocity.dot(bodyForward);
    dt.shiftUp(forwardSpeedMs, eng.rpm);
  };

  const manualShiftDown = () => {
    const dt = drivetrainRef.current;
    const body = rigidBodyRef.current;
    const eng = engineRef.current;
    if (!dt || !body || !eng) return;
    const bodyForward = SCRATCH_VEC3_A.set(0, 0, 1).applyQuaternion(body.quaternion);
    const forwardSpeedMs = body.linearVelocity.dot(bodyForward);
    dt.shiftDown(forwardSpeedMs, eng.rpm);
  };

  useEffect(() => {
    if (!mountRef.current) return;

    // 1. Scene, Camera, Renderer Setup
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xdce7e8);
    scene.fog = new THREE.FogExp2(0xdce7e8, 0.008);

    const camera = new THREE.PerspectiveCamera(
      45,
      mountRef.current.clientWidth / mountRef.current.clientHeight,
      0.1,
      1000
    );
    camera.position.set(0, 2.8, -7.5);
    cameraRef.current = camera;

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(mountRef.current.clientWidth, mountRef.current.clientHeight);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFShadowMap;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.05;
    mountRef.current.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.maxPolarAngle = Math.PI / 2 - 0.04;
    controls.minPolarAngle = 0.05;
    controls.minDistance = 2.5;
    controls.maxDistance = 50.0;
    controls.target.set(0, 0.85, 0);
    controlsRef.current = controls;

    // 2. Lighting Setup
    const sunLight = new THREE.DirectionalLight(0xfff5e6, 2.0);
    sunLight.position.set(45, 60, -35);
    sunLight.castShadow = true;
    sunLight.shadow.mapSize.width = 2048;
    sunLight.shadow.mapSize.height = 2048;
    sunLight.shadow.camera.near = 0.5;
    sunLight.shadow.camera.far = 160;
    sunLight.shadow.camera.left = -60;
    sunLight.shadow.camera.right = 60;
    sunLight.shadow.camera.top = 60;
    sunLight.shadow.camera.bottom = -60;
    sunLight.shadow.bias = -0.0005;
    scene.add(sunLight);

    const hemiLight = new THREE.HemisphereLight(0xe5eff5, 0xc2c7b5, 1.2);
    scene.add(hemiLight);

    // 3. Test Ground Sandbox
    const testGround = new TestGround();
    scene.add(testGround.mesh);
    groundRef.current = testGround;

    // 4. Vehicle Physics Modules
    const rigidBody = new RigidBody({ mass: 1800 });
    rigidBodyRef.current = rigidBody;

    const suspension = new SuspensionSystem();
    suspensionRef.current = suspension;

    const wheelsDyn = [
      new WheelDynamics('FL'),
      new WheelDynamics('FR'),
      new WheelDynamics('RL'),
      new WheelDynamics('RR'),
    ];
    wheelsDynRef.current = wheelsDyn;

    const steering = new SteeringSystem();
    steeringRef.current = steering;

    const engine = new Engine();
    engineRef.current = engine;

    const drivetrain = new Drivetrain();
    drivetrainRef.current = drivetrain;

    const brakes = new BrakeSystem();
    brakesRef.current = brakes;

    const aero = new Aerodynamics();
    aeroRef.current = aero;

    const chassisContact = new ChassisContactSystem();
    chassisContactRef.current = chassisContact;

    const audio = new VehicleAudio();
    audioRef.current = audio;

    const handleAudioUnlock = () => {
      audio.unlock();
      window.removeEventListener('keydown', handleAudioUnlock);
      window.removeEventListener('pointerdown', handleAudioUnlock);
    };
    window.addEventListener('keydown', handleAudioUnlock, { once: true });
    window.addEventListener('pointerdown', handleAudioUnlock, { once: true });

    (window as unknown as { __VEHICLE_AUDIO__?: unknown }).__VEHICLE_AUDIO__ = audio;
    (window as unknown as { __DRIVETRAIN__?: unknown }).__DRIVETRAIN__ = drivetrain;
    (window as unknown as { __ENGINE__?: unknown }).__ENGINE__ = engine;

    // 5. 3D Visual Mesh Model
    const visualModel = new VehicleVisualModel();
    scene.add(visualModel.group);
    visualModelRef.current = visualModel;

    // Set initial position to Station 0
    resetToStation(0);

    const input = new InputManager();
    inputRef.current = input;

    // 6. Fixed 120 Hz Loop with Clean Minimal Render
    const fixedLoop = new FixedTimestepLoop(
      (dt: number) => {
        // Save previous physics state for render interpolation
        prevBodyPos.current.copy(rigidBody.position);
        prevBodyQuat.current.copy(rigidBody.quaternion);

        for (let i = 0; i < 4; i++) {
          prevWheelPos.current[i].copy(suspension.wheels[i].wheelWorldPosition);
          prevSteerAngle.current[i] = wheelsDyn[i].steerAngle;
          prevSpinAngle.current[i] = wheelsDyn[i].spinAngle;
        }

        rigidBody.clearAccumulators();

        // Sideways push test impulse
        if (pendingSidewaysPushRef.current) {
          pendingSidewaysPushRef.current = false;
          const bodyRight = SCRATCH_VEC3_A.set(1, 0, 0).applyQuaternion(rigidBody.quaternion);
          rigidBody.linearVelocity.addScaledVector(bodyRight, 4.0);
          rigidBody.angularVelocity.y += 0.5;
        }

        // Steering command
        let steerInput = 0;
        if (input.isDown('a') || input.isDown('arrowleft')) steerInput += 1;
        if (input.isDown('d') || input.isDown('arrowright')) steerInput -= 1;

        const bodyForwardVec = SCRATCH_VEC3_A.set(0, 0, 1).applyQuaternion(rigidBody.quaternion);
        const forwardSpeedMs = rigidBody.linearVelocity.dot(bodyForwardVec);

        steering.update(steerInput, forwardSpeedMs, dt);
        wheelsDyn[0].steerAngle = steering.steerAngleFL;
        wheelsDyn[1].steerAngle = steering.steerAngleFR;
        const steerAngles = [steering.steerAngleFL, steering.steerAngleFR, 0, 0];

        // Suspension multi-ray evaluation with progressive compression and asymmetric rebound damping
        suspension.update(rigidBody, testGround, dt, steerAngles);

        // Aerodynamics forces
        aero.update(rigidBody);

        // Pedals & Drive / Brake inputs
        const throttleKey = input.isDown('w') || input.isDown('arrowup');
        const brakeKey = input.isDown('s') || input.isDown('arrowdown');
        const handbrakeKey = input.isDown(' ') || handbrakeLatchedRef.current;

        let targetThrottle = 0;
        let brakePedal = 0;

        if (drivetrain.mode === 'auto') {
          if (drivetrain.currentGear >= 1) {
            if (throttleKey) targetThrottle = 1.0;
            if (brakeKey) {
              if (forwardSpeedMs > 0.4) {
                brakePedal = 1.0;
              } else {
                if (drivetrain.targetGear !== -1) {
                  drivetrain.requestGear(-1, forwardSpeedMs, engine.rpm);
                }
              }
            }
          } else if (drivetrain.currentGear === -1) {
            if (brakeKey && !drivetrain.isShifting) targetThrottle = 0.85;
            if (throttleKey) {
              if (forwardSpeedMs < -0.4) {
                brakePedal = 1.0;
              } else {
                if (drivetrain.targetGear !== 1) {
                  drivetrain.requestGear(1, forwardSpeedMs, engine.rpm);
                }
              }
            }
          } else {
            if (throttleKey) drivetrain.requestGear(1, forwardSpeedMs, engine.rpm);
            if (brakeKey) drivetrain.requestGear(-1, forwardSpeedMs, engine.rpm);
          }
        } else {
          if (throttleKey) targetThrottle = 1.0;
          if (brakeKey) brakePedal = 1.0;
        }

        const throttlePedal = targetThrottle;

        // Transmission & Engine Dynamics
        drivetrain.update(engine, wheelsDyn, forwardSpeedMs, dt);

        engine.update(
          throttlePedal,
          drivetrain.clutchReactionTorque,
          drivetrain.clutchEngagement,
          drivetrain.targetDrivetrainRpm,
          dt
        );

        drivetrain.update(engine, wheelsDyn, forwardSpeedMs, dt, false);

        // Service & Handbrake System
        brakes.update(brakePedal, handbrakeKey, wheelsDyn, Math.abs(forwardSpeedMs), dt);

        // 4-Wheel Contact Kinematics, Pacejka Tires, Soft-Soil Sinkage & Bulldozing
        for (let i = 0; i < 4; i++) {
          const suspWheel = suspension.wheels[i];
          const dynWheel = wheelsDyn[i];

          const surfaceQuery = testGround.getHeightNormalSurface(
            suspWheel.contactPoint.x,
            suspWheel.contactPoint.z
          );

          dynWheel.update(
            suspWheel,
            rigidBody,
            drivetrain.wheelTorques[i],
            brakes.wheelBrakeTorques[i],
            surfaceQuery.surface,
            dt,
            brakes.isHandbrakeActive && (i === 2 || i === 3)
          );
        }

        // Chassis 12-Point Ground Contacts & Safety-Net Correction
        chassisContact.update(rigidBody, testGround, dt, suspension, wheelsDyn);

        // Rigid Body Symplectic Integration
        rigidBody.integrate(dt);

        // Post-Integration Boundary Enforcement
        chassisContact.postIntegrate(rigidBody, testGround, suspension, wheelsDyn);

        currBodyPos.current.copy(rigidBody.position);
        currBodyQuat.current.copy(rigidBody.quaternion);

        for (let i = 0; i < 4; i++) {
          currWheelPos.current[i].copy(suspension.wheels[i].wheelWorldPosition);
          currSteerAngle.current[i] = wheelsDyn[i].steerAngle;
          currSpinAngle.current[i] = wheelsDyn[i].spinAngle;
        }
      },
      (alpha: number, _frameDt: number) => {
        // Hotkey controls (minimal clean HUD retained, debug HUD removed)
        if (input.wasJustPressed('r')) {
          resetToStation(stationIndexRef.current);
        }
        if (input.wasJustPressed('t')) {
          nextStation();
        }
        if (input.wasJustPressed('p')) {
          applySidewaysPush();
        }
        if (input.wasJustPressed('j')) {
          applyRolloverPose();
        }
        if (input.wasJustPressed('k')) {
          applySafetyNetTestPose();
        }
        if (input.wasJustPressed('q')) {
          manualShiftDown();
        }
        if (input.wasJustPressed('e')) {
          manualShiftUp();
        }
        if (input.wasJustPressed('m')) {
          toggleTransmissionMode();
        }
        if (input.wasJustPressed('g')) {
          toggleLayout();
        }
        if (input.wasJustPressed('b')) {
          toggleAbs();
        }
        if (input.wasJustPressed('l')) {
          toggleRangeCase();
        }
        if (input.wasJustPressed(' ')) {
          handbrakeLatchedRef.current = !handbrakeLatchedRef.current;
        }

        // Render Interpolation
        renderBodyPos.current.lerpVectors(prevBodyPos.current, currBodyPos.current, alpha);
        renderBodyQuat.current.slerpQuaternions(prevBodyQuat.current, currBodyQuat.current, alpha);

        for (let i = 0; i < 4; i++) {
          renderWheelPos.current[i].lerpVectors(prevWheelPos.current[i], currWheelPos.current[i], alpha);
          renderSteerAngle.current[i] = lerp(prevSteerAngle.current[i], currSteerAngle.current[i], alpha);
          renderSpinAngle.current[i] = lerp(prevSpinAngle.current[i], currSpinAngle.current[i], alpha);
        }

        visualModel.update(
          rigidBody,
          suspension,
          wheelsDyn,
          renderBodyPos.current,
          renderBodyQuat.current,
          renderWheelPos.current,
          renderSteerAngle.current,
          renderSpinAngle.current
        );

        // Camera Follow with User-Controlled Orbit
        const carFocusX = renderBodyPos.current.x;
        const carFocusY = renderBodyPos.current.y + 0.85;
        const carFocusZ = renderBodyPos.current.z;

        const deltaX = carFocusX - prevCarFollowPos.current.x;
        const deltaY = carFocusY - prevCarFollowPos.current.y;
        const deltaZ = carFocusZ - prevCarFollowPos.current.z;

        camera.position.x += deltaX;
        camera.position.y += deltaY;
        camera.position.z += deltaZ;

        controls.target.x += deltaX;
        controls.target.y += deltaY;
        controls.target.z += deltaZ;

        if (camera.position.y < 0.35) {
          camera.position.y = 0.35;
        }

        prevCarFollowPos.current.set(carFocusX, carFocusY, carFocusZ);
        controls.update();
        renderer.render(scene, camera);

        // Procedural Audio Engine Update & Shift FX
        const bodyFwdVec = SCRATCH_VEC3_A.set(0, 0, 1).applyQuaternion(rigidBody.quaternion);
        const fwdSpeedKmh = Math.abs(rigidBody.linearVelocity.dot(bodyFwdVec)) * 3.6;

        if (drivetrain.currentGear !== lastGearRef.current) {
          audio.playShiftClunk(0.65);
          lastGearRef.current = drivetrain.currentGear;
        }

        audio.update(
          engine.rpm,
          engine.throttle,
          _frameDt,
          drivetrain.isShifting,
          fwdSpeedKmh,
          drivetrain.rangeMode
        );

        // Throttled Driving HUD updates (~20 Hz) for buttery reading without React render bottle-neck
        const now = performance.now();
        if (now - lastHudUpdateRef.current > 50) {
          lastHudUpdateRef.current = now;
          setHudState({
            speedKmh: fwdSpeedKmh,
            currentGear: drivetrain.currentGear,
            mode: drivetrain.mode,
            rangeMode: drivetrain.rangeMode,
          });
        }
      }
    );

    fixedLoop.start();

    const handleResize = () => {
      if (!mountRef.current) return;
      const width = mountRef.current.clientWidth;
      const height = mountRef.current.clientHeight;
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      renderer.setSize(width, height);
    };

    window.addEventListener('resize', handleResize);

    return () => {
      fixedLoop.stop();
      window.removeEventListener('resize', handleResize);
      window.removeEventListener('keydown', handleAudioUnlock);
      window.removeEventListener('pointerdown', handleAudioUnlock);
      delete (window as unknown as { __VEHICLE_AUDIO__?: unknown }).__VEHICLE_AUDIO__;
      delete (window as unknown as { __DRIVETRAIN__?: unknown }).__DRIVETRAIN__;
      delete (window as unknown as { __ENGINE__?: unknown }).__ENGINE__;
      audio.dispose();
      input.dispose();
      renderer.dispose();
      if (mountRef.current && renderer.domElement) {
        mountRef.current.removeChild(renderer.domElement);
      }
    };
  }, []);

  return (
    <div className="relative w-screen h-screen overflow-hidden bg-stone-900 select-none">
      <div ref={mountRef} className="absolute inset-0 w-full h-full" />

      {/* Minimal Driving HUD Overlay (Gear, Speed, Mode, Range) */}
      <HUD
        speedKmh={hudState.speedKmh}
        currentGear={hudState.currentGear}
        mode={hudState.mode}
        rangeMode={hudState.rangeMode}
      />

      {/* Subtle In-Game Toast (Clean non-debug feedback) */}
      {hudMessage && (
        <div className="pointer-events-none fixed top-8 left-1/2 -translate-x-1/2 z-50 transition-opacity">
          <div className="flex items-center gap-2 rounded-xl border border-stone-600 bg-stone-900/90 px-4 py-2 shadow-2xl backdrop-blur-md">
            <span className="font-mono text-xs font-semibold text-stone-200 uppercase tracking-wider">
              {hudMessage}
            </span>
          </div>
        </div>
      )}
    </div>
  );
}

export { App };
