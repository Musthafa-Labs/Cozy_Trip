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
import { TestGround, TestStation } from './world/testGround';
import { VehicleVisualModel } from './vehicle/vehicleModel';
import { DebugVisualizer } from './render/DebugVisualizer';
import { FixedTimestepLoop } from './core/loop';
import { InputManager } from './core/input';
import { DebugOverlay, TelemetryData } from './ui/DebugOverlay';
import { SCRATCH_VEC3_A, clamp, lerp } from './utils/math';

export default function App() {
  const mountRef = useRef<HTMLDivElement>(null);

  // UI state
  const [debugVisible, setDebugVisible] = useState<boolean>(true);
  const [telemetry, setTelemetry] = useState<TelemetryData | null>(null);

  // Live simulation references
  const rigidBodyRef = useRef<RigidBody | null>(null);
  const suspensionRef = useRef<SuspensionSystem | null>(null);
  const wheelsDynRef = useRef<WheelDynamics[]>([]);
  const steeringRef = useRef<SteeringSystem | null>(null);
  const engineRef = useRef<Engine | null>(null);
  const drivetrainRef = useRef<Drivetrain | null>(null);
  const brakesRef = useRef<BrakeSystem | null>(null);
  const aeroRef = useRef<Aerodynamics | null>(null);
  const groundRef = useRef<TestGround | null>(null);
  const visualModelRef = useRef<VehicleVisualModel | null>(null);
  const debugVisRef = useRef<DebugVisualizer | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const controlsRef = useRef<OrbitControls | null>(null);
  const prevCarFollowPos = useRef(new THREE.Vector3(0, 0.85, 0));
  const stationIndexRef = useRef<number>(0);
  const inputRef = useRef<InputManager | null>(null);

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

  const renderBodyPos = useRef(new THREE.Vector3());
  const renderBodyQuat = useRef(new THREE.Quaternion());
  const renderWheelPos = useRef([new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()]);
  const renderSteerAngle = useRef([0, 0, 0, 0]);
  const renderSpinAngle = useRef([0, 0, 0, 0]);

  // Reset vehicle to designated test station
  const resetToStation = (index: number) => {
    const ground = groundRef.current;
    const body = rigidBodyRef.current;
    const wheels = wheelsDynRef.current;
    const controls = controlsRef.current;
    const susp = suspensionRef.current;
    const engine = engineRef.current;
    const drivetrain = drivetrainRef.current;
    const brakes = brakesRef.current;
    const aero = aeroRef.current;
    if (!ground || !body || !susp) return;

    stationIndexRef.current = ((index % ground.stations.length) + ground.stations.length) % ground.stations.length;
    const targetStation = ground.stations[stationIndexRef.current];

    body.resetTo(targetStation.position, targetStation.headingRad);
    wheels.forEach((w) => w.reset());
    if (engine) engine.reset();
    if (drivetrain) drivetrain.reset();
    if (brakes) brakes.reset();
    if (aero) aero.reset();

    // Sync previous and current interpolation history to prevent jump on reset
    prevBodyPos.current.copy(body.position);
    currBodyPos.current.copy(body.position);
    prevBodyQuat.current.copy(body.quaternion);
    currBodyQuat.current.copy(body.quaternion);

    susp.update(body, ground, 1 / 120);

    for (let i = 0; i < 4; i++) {
      prevWheelPos.current[i].copy(susp.wheels[i].wheelWorldPosition);
      currWheelPos.current[i].copy(susp.wheels[i].wheelWorldPosition);
      prevSteerAngle.current[i] = 0;
      currSteerAngle.current[i] = 0;
      prevSpinAngle.current[i] = 0;
      currSpinAngle.current[i] = 0;
    }

    const followTarget = new THREE.Vector3(body.position.x, body.position.y + 0.85, body.position.z);
    if (controls) {
      controls.target.copy(followTarget);
      if (cameraRef.current) {
        // Reset to default third-person view: behind vehicle (-Z), elevated (+Y)
        cameraRef.current.position.set(
          followTarget.x,
          followTarget.y + 1.95,
          followTarget.z - 7.5
        );
      }
      controls.update();
    }
    prevCarFollowPos.current.copy(followTarget);
  };

  const nextStation = () => {
    resetToStation(stationIndexRef.current + 1);
  };

  // Lateral push impulse for roll & stability verification
  const applySidewaysPush = () => {
    const body = rigidBodyRef.current;
    if (!body) return;
    const pushForce = new THREE.Vector3(14000, 0, 0); // 14 kN lateral impulse
    const pushPoint = body.position.clone().add(new THREE.Vector3(0, 0.4, 0));
    body.applyForceAtWorldPoint(pushForce, pushPoint);
  };

  // Toggle Drivetrain Layout (AWD -> RWD -> FWD)
  const toggleLayout = () => {
    const dt = drivetrainRef.current;
    if (!dt) return;
    if (dt.layout === 'AWD') dt.layout = 'RWD';
    else if (dt.layout === 'RWD') dt.layout = 'FWD';
    else dt.layout = 'AWD';
  };

  // Toggle Transmission Mode (Auto / Manual)
  const toggleTransmissionMode = () => {
    const dt = drivetrainRef.current;
    if (!dt) return;
    dt.mode = dt.mode === 'auto' ? 'manual' : 'auto';
  };

  // Toggle ABS On/Off
  const toggleAbs = () => {
    const b = brakesRef.current;
    if (!b) return;
    b.config.absEnabled = !b.config.absEnabled;
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
    // Default third-person perspective: behind vehicle (-Z), slightly elevated (+Y), looking down at full car
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
    controls.maxPolarAngle = Math.PI / 2 - 0.04; // Prevent dipping beneath the ground
    controls.minPolarAngle = 0.05;
    controls.minDistance = 2.5;                  // Sensible minimum zoom (close up)
    controls.maxDistance = 50.0;                 // Sensible maximum zoom
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

    // 4. Vehicle Subsystems
    const rigidBody = new RigidBody();
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

    const visualModel = new VehicleVisualModel();
    scene.add(visualModel.group);
    visualModelRef.current = visualModel;

    const debugVisualizer = new DebugVisualizer();
    scene.add(debugVisualizer.group);
    debugVisRef.current = debugVisualizer;

    resetToStation(0);

    // 5. Input System
    const input = new InputManager();
    inputRef.current = input;

    // 6. 120 Hz Physics Simulation Loop
    let lastUiUpdate = 0;

    const fixedLoop = new FixedTimestepLoop(
      (dt: number) => {
        // Record previous physics state before stepping
        prevBodyPos.current.copy(rigidBody.position);
        prevBodyQuat.current.copy(rigidBody.quaternion);

        for (let i = 0; i < 4; i++) {
          prevWheelPos.current[i].copy(suspension.wheels[i].wheelWorldPosition);
          prevSteerAngle.current[i] = wheelsDyn[i].steerAngle;
          prevSpinAngle.current[i] = wheelsDyn[i].spinAngle;
        }

        // =====================================================================
        // A. Steering Input & Ackermann Update
        // A / Left gives positive steer command (+1) => steering LEFT
        // D / Right gives negative steer command (-1) => steering RIGHT
        // =====================================================================
        let steerInput = 0;
        if (input.isDown('a') || input.isDown('arrowleft')) steerInput += 1;
        if (input.isDown('d') || input.isDown('arrowright')) steerInput -= 1;

        // Vehicle forward speed along body forward vector (+Z)
        const bodyForward = SCRATCH_VEC3_A.set(0, 0, 1).applyQuaternion(rigidBody.quaternion);
        const forwardSpeedMs = rigidBody.linearVelocity.dot(bodyForward);

        steering.update(steerInput, forwardSpeedMs, dt);
        wheelsDyn[0].steerAngle = steering.steerAngleFL;
        wheelsDyn[1].steerAngle = steering.steerAngleFR;
        wheelsDyn[2].steerAngle = 0;
        wheelsDyn[3].steerAngle = 0;

        // =====================================================================
        // B. Aerodynamics Update (Fdrag = 0.5 * rho * Cd * A * v^2)
        // =====================================================================
        aero.update(rigidBody);

        // =====================================================================
        // C. Powertrain & Transmission Input Mapping
        // =====================================================================
        const throttleKey = input.isDown('w') || input.isDown('arrowup');
        const brakeKey = input.isDown('s') || input.isDown('arrowdown');
        const handbrakePressed = input.isDown(' ');

        let throttlePedal = 0;
        let brakePedal = 0;

        if (drivetrain.mode === 'auto') {
          // Automatic mode: S acts as brake when moving forward, or reverse when stopped
          if (drivetrain.currentGear >= 1) {
            // Forward gear
            if (throttleKey) {
              if (!drivetrain.isShifting) throttlePedal = 1.0;
            }
            if (brakeKey) {
              if (forwardSpeedMs > 0.4) {
                brakePedal = 1.0;
              } else {
                // At stop, shift into Reverse
                if (drivetrain.targetGear !== -1) {
                  drivetrain.requestGear(-1, forwardSpeedMs, engine.rpm);
                }
              }
            }
          } else if (drivetrain.currentGear === -1) {
            // Reverse gear
            if (brakeKey) {
              if (!drivetrain.isShifting) throttlePedal = 0.85; // Reverse accelerator
            }
            if (throttleKey) {
              if (forwardSpeedMs < -0.4) {
                brakePedal = 1.0; // Brake backwards motion
              } else {
                // At stop, shift back into 1st forward gear
                if (drivetrain.targetGear !== 1) {
                  drivetrain.requestGear(1, forwardSpeedMs, engine.rpm);
                }
              }
            }
          } else {
            // Neutral
            if (throttleKey) drivetrain.requestGear(1, forwardSpeedMs, engine.rpm);
            if (brakeKey) drivetrain.requestGear(-1, forwardSpeedMs, engine.rpm);
          }
        } else {
          // Manual mode: W is strictly throttle, S is strictly service brakes
          if (throttleKey) throttlePedal = 1.0;
          if (brakeKey) brakePedal = 1.0;
        }

        // =====================================================================
        // D. Transmission & Engine Dynamics Integration
        // =====================================================================
        // First pass: update transmission shift timing and clutch state from wheel feedback
        drivetrain.update(engine, wheelsDyn, forwardSpeedMs, dt);

        // Update engine RPM, torque curve evaluation, rev limiter, and idle governor
        engine.update(
          throttlePedal,
          drivetrain.clutchReactionTorque,
          drivetrain.clutchEngagement,
          drivetrain.targetDrivetrainRpm,
          dt
        );

        // Second pass: compute driveshaft torque and split to driven wheels with final engine output
        drivetrain.update(engine, wheelsDyn, forwardSpeedMs, dt, false);

        // =====================================================================
        // E. Braking System & ABS Modulation
        // =====================================================================
        brakes.update(brakePedal, handbrakePressed, wheelsDyn, Math.abs(forwardSpeedMs), dt);

        // =====================================================================
        // F. Suspension System Update (Raycasts & Spring Normal Loads)
        // =====================================================================
        suspension.update(rigidBody, testGround, dt);

        // =====================================================================
        // G. 4-Wheel Contact Kinematics, Pacejka Tires & Wheel Integration
        // =====================================================================
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
            dt
          );
        }

        // =====================================================================
        // H. Rigid Body Symplectic Integration
        // =====================================================================
        rigidBody.integrate(dt);

        // Store new current state after physics step
        currBodyPos.current.copy(rigidBody.position);
        currBodyQuat.current.copy(rigidBody.quaternion);

        for (let i = 0; i < 4; i++) {
          currWheelPos.current[i].copy(suspension.wheels[i].wheelWorldPosition);
          currSteerAngle.current[i] = wheelsDyn[i].steerAngle;
          currSpinAngle.current[i] = wheelsDyn[i].spinAngle;
        }
      },
      (alpha: number, _frameDt: number) => {
        // Hotkey commands
        if (input.wasJustPressed('r')) {
          resetToStation(stationIndexRef.current);
        }
        if (input.wasJustPressed('t')) {
          nextStation();
        }
        if (input.wasJustPressed('p')) {
          applySidewaysPush();
        }
        if (input.wasJustPressed('f3')) {
          setDebugVisible((prev) => !prev);
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

        // =====================================================================
        // PART C.3: Render Interpolation using alpha
        // Blends previous and current physics state (zero allocations)
        // =====================================================================
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

        debugVisualizer.update(rigidBody, suspension, wheelsDyn);

        // =====================================================================
        // CAMERA FOLLOW WITH USER-CONTROLLED ORBIT (ANGLE RETENTION)
        // Moves camera and orbit focus target in lockstep with car displacement.
        // User's chosen spherical angle & zoom distance are 100% preserved.
        // =====================================================================
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

        // Ground collision safeguard
        if (camera.position.y < 0.35) {
          camera.position.y = 0.35;
        }

        prevCarFollowPos.current.set(carFocusX, carFocusY, carFocusZ);

        controls.update();

        renderer.render(scene, camera);

        // React telemetry update (~20 Hz)
        const now = performance.now();
        if (now - lastUiUpdate > 50) {
          lastUiUpdate = now;

          const euler = new THREE.Euler().setFromQuaternion(rigidBody.quaternion, 'YXZ');
          const pitchDeg = THREE.MathUtils.radToDeg(euler.x);
          const rollDeg = THREE.MathUtils.radToDeg(euler.z);

          const speedMs = rigidBody.linearVelocity.length();
          const speedKmh = speedMs * 3.6;

          const com = SCRATCH_VEC3_A;
          rigidBody.getWorldCOM(com);

          // Headings and sideslip
          const bodyFwd = new THREE.Vector3(0, 0, 1).applyQuaternion(rigidBody.quaternion);
          const headingDeg = THREE.MathUtils.radToDeg(Math.atan2(bodyFwd.x, bodyFwd.z));

          const horizSpeed = Math.hypot(rigidBody.linearVelocity.x, rigidBody.linearVelocity.z);
          const velocityDirDeg = horizSpeed > 0.1
            ? THREE.MathUtils.radToDeg(Math.atan2(rigidBody.linearVelocity.x, rigidBody.linearVelocity.z))
            : headingDeg;

          let bodySideslipDeg = velocityDirDeg - headingDeg;
          while (bodySideslipDeg > 180) bodySideslipDeg -= 360;
          while (bodySideslipDeg < -180) bodySideslipDeg += 360;

          // Yaw rate (turning left = positive)
          const yawRateDeg = THREE.MathUtils.radToDeg(rigidBody.angularVelocity.y);

          // Steer Command string
          let steerCmd: 'LEFT' | 'RIGHT' | 'CENTER' = 'CENTER';
          if (steering.currentVirtualAngle > 0.008) steerCmd = 'LEFT';
          else if (steering.currentVirtualAngle < -0.008) steerCmd = 'RIGHT';

          const steerAngleFLDeg = THREE.MathUtils.radToDeg(wheelsDyn[0].steerAngle);
          const steerAngleFRDeg = THREE.MathUtils.radToDeg(wheelsDyn[1].steerAngle);

          const avgDriveT = (drivetrain.wheelTorques[0] + drivetrain.wheelTorques[1] + drivetrain.wheelTorques[2] + drivetrain.wheelTorques[3]) / 4;
          const avgBrakeT = (brakes.wheelBrakeTorques[0] + brakes.wheelBrakeTorques[1] + brakes.wheelBrakeTorques[2] + brakes.wheelBrakeTorques[3]) / 4;

          setTelemetry({
            speedKmh,
            speedMs,
            pitchDeg,
            rollDeg,
            comHeight: com.y,
            nanResets: rigidBody.nanResetCount,
            wheelsSusp: suspension.wheels,
            wheelsDyn: wheelsDyn,
            currentStation: testGround.stations[stationIndexRef.current],
            stationIndex: stationIndexRef.current,
            totalStations: testGround.stations.length,
            driveTorquePerWheel: avgDriveT,
            brakeTorquePerWheel: avgBrakeT,
            handbrakeActive: brakes.isHandbrakeActive,
            virtualSteerDeg: THREE.MathUtils.radToDeg(steering.currentVirtualAngle),
            steerCmd,
            steerAngleFLDeg,
            steerAngleFRDeg,
            yawRateDeg,
            headingDeg,
            velocityDirDeg,
            bodySideslipDeg,

            // STEP 4 Powertrain, Brakes & Aero Telemetry
            engineRpm: engine.rpm,
            engineTorque: engine.outputTorque,
            throttlePct: engine.throttle * 100,
            brakePct: brakes.pedalPosition * 100,
            currentGear: drivetrain.currentGear,
            targetGear: drivetrain.targetGear,
            isShifting: drivetrain.isShifting,
            transmissionMode: drivetrain.mode,
            drivetrainLayout: drivetrain.layout,
            clutchEngagement: drivetrain.clutchEngagement,
            absActive: [...brakes.absActive],
            absGlobalActive: brakes.absGlobalActive,
            absEnabled: brakes.config.absEnabled,
            aeroDragN: aero.dragForceMagnitude,
            aeroDownforceN: aero.downforceMagnitude,
          });
        }

        input.endFrame();
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

      <DebugOverlay
        visible={debugVisible}
        onToggleVisible={() => {
          setDebugVisible((prev) => !prev);
          if (debugVisRef.current) {
            debugVisRef.current.visible = !debugVisible;
          }
        }}
        telemetry={telemetry}
        onReset={() => resetToStation(stationIndexRef.current)}
        onNextStation={nextStation}
        onSelectStation={resetToStation}
        onApplyPush={applySidewaysPush}
        onToggleLayout={toggleLayout}
        onToggleTransMode={toggleTransmissionMode}
        onToggleAbs={toggleAbs}
        onShiftUp={manualShiftUp}
        onShiftDown={manualShiftDown}
      />
    </div>
  );
}
