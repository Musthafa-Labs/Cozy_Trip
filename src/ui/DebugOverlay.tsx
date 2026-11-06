/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import React, { useRef, useEffect } from 'react';
import { WheelState } from '../vehicle/suspension';
import { WheelDynamics } from '../vehicle/wheel';
import { TestStation } from '../world/testGround';
import { SURFACE_PACEJKA, SurfaceType } from '../vehicle/tire';
import { DrivetrainLayout, TransmissionMode } from '../vehicle/drivetrain';

export interface TelemetryData {
  speedKmh: number;
  speedMs: number;
  pitchDeg: number;
  rollDeg: number;
  comHeight: number;
  nanResets: number;
  wheelsSusp: WheelState[];
  wheelsDyn: WheelDynamics[];
  currentStation: TestStation;
  stationIndex: number;
  totalStations: number;
  driveTorquePerWheel: number;
  brakeTorquePerWheel: number;
  handbrakeActive: boolean;
  virtualSteerDeg: number;

  // Steering & Chassis Kinematics
  steerCmd: 'LEFT' | 'RIGHT' | 'CENTER';
  steerAngleFLDeg: number;  // deg (positive = left)
  steerAngleFRDeg: number;  // deg (positive = left)
  yawRateDeg: number;       // deg/s (positive = turning left)
  headingDeg: number;       // deg (body heading angle)
  velocityDirDeg: number;   // deg (velocity direction angle)
  bodySideslipDeg: number;  // deg (body sideslip angle)

  // STEP 4: Powertrain, Brakes, Aerodynamics
  engineRpm: number;
  engineTorque: number;
  throttlePct: number;
  brakePct: number;
  currentGear: number;       // -1 = R, 0 = N, 1..6 = Gears
  targetGear: number;
  isShifting: boolean;
  transmissionMode: TransmissionMode;
  drivetrainLayout: DrivetrainLayout;
  clutchEngagement: number;  // 0 .. 1.0
  absActive: [boolean, boolean, boolean, boolean];
  absGlobalActive: boolean;
  absEnabled: boolean;
  aeroDragN: number;
  aeroDownforceN: number;
}

interface DebugOverlayProps {
  visible: boolean;
  onToggleVisible: () => void;
  telemetry: TelemetryData | null;
  onReset: () => void;
  onNextStation: () => void;
  onSelectStation: (index: number) => void;
  onApplyPush: () => void;
  onToggleLayout?: () => void;
  onToggleTransMode?: () => void;
  onToggleAbs?: () => void;
  onShiftUp?: () => void;
  onShiftDown?: () => void;
}

export const DebugOverlay: React.FC<DebugOverlayProps> = ({
  visible,
  onToggleVisible,
  telemetry,
  onReset,
  onNextStation,
  onSelectStation,
  onApplyPush,
  onToggleLayout,
  onToggleTransMode,
  onToggleAbs,
  onShiftUp,
  onShiftDown,
}) => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  useEffect(() => {
    if (!visible || !telemetry || !canvasRef.current) return;
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const width = canvas.width;
    const height = canvas.height;
    ctx.clearRect(0, 0, width, height);

    ctx.fillStyle = '#1e2221';
    ctx.fillRect(0, 0, width, height);

    ctx.strokeStyle = '#323a37';
    ctx.lineWidth = 1;

    const midX = width / 2;
    const midY = height / 2;
    ctx.beginPath();
    ctx.moveTo(0, midY);
    ctx.lineTo(width, midY);
    ctx.moveTo(midX, 0);
    ctx.lineTo(midX, height);
    ctx.stroke();

    const flDyn = telemetry.wheelsDyn[0];
    const flSusp = telemetry.wheelsSusp[0];
    const surface: SurfaceType = flDyn.surface ?? 'dirt';
    const coeffs = SURFACE_PACEJKA[surface] ?? SURFACE_PACEJKA.dirt;
    const Fz = Math.max(100, flSusp.normalLoad);
    const Dy = coeffs.muY * Fz;

    ctx.strokeStyle = '#4ade80';
    ctx.lineWidth = 2;
    ctx.beginPath();

    const maxAlphaDeg = 25;
    const maxFy = Math.max(4500, Dy * 1.15);

    for (let xPix = 0; xPix <= width; xPix += 2) {
      const alphaDeg = ((xPix - midX) / (width / 2)) * maxAlphaDeg;
      const alphaRad = (alphaDeg * Math.PI) / 180;
      const ByA = coeffs.By * alphaRad;
      const Fy = -Dy * Math.sin(coeffs.Cy * Math.atan(ByA - coeffs.Ey * (ByA - Math.atan(ByA))));

      const yPix = midY - (Fy / maxFy) * (height / 2 - 8);
      if (xPix === 0) ctx.moveTo(xPix, yPix);
      else ctx.lineTo(xPix, yPix);
    }
    ctx.stroke();

    const curAlphaDeg = flDyn.slipAngleDeg;
    const curFy = flDyn.Fy;
    const curXPix = midX + (curAlphaDeg / maxAlphaDeg) * (width / 2);
    const curYPix = midY - (curFy / maxFy) * (height / 2 - 8);

    ctx.fillStyle = '#f97316';
    ctx.beginPath();
    ctx.arc(Math.max(4, Math.min(width - 4, curXPix)), Math.max(4, Math.min(height - 4, curYPix)), 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.5;
    ctx.stroke();
  }, [visible, telemetry]);

  if (!telemetry) return null;

  // Gear label helper
  const getGearLabel = (g: number): string => {
    if (g === -1) return 'R';
    if (g === 0) return 'N';
    return `${g}`;
  };

  const rpmProgress = Math.min(100, Math.max(0, ((telemetry.engineRpm - 800) / (7200 - 800)) * 100));
  const isRedline = telemetry.engineRpm >= 6500;

  return (
    <div className="pointer-events-none fixed inset-0 z-50 flex flex-col justify-between p-4 font-sans text-neutral-800 antialiased select-none">
      {/* Top Bar: Station Info & Powertrain HUD Cluster */}
      <div className="flex flex-wrap items-start justify-between gap-3">
        {/* Left: Track info card */}
        <div className="pointer-events-auto rounded-xl border border-stone-300/80 bg-stone-50/95 p-3.5 shadow-md backdrop-blur-md transition-all sm:max-w-md">
          <div className="flex items-center justify-between gap-2 border-b border-stone-200 pb-2">
            <span className="text-xs font-bold uppercase tracking-wider text-emerald-800">
              Straight Test Road • 5.0 km
            </span>
            <div className="flex items-center gap-1.5">
              <span className="inline-block h-2 w-2 rounded-full bg-emerald-500 animate-pulse"></span>
              <span className="text-[11px] font-semibold text-stone-600">120 Hz Pacejka MF</span>
            </div>
          </div>
          <h2 className="mt-1.5 text-base font-semibold text-stone-900">High-Speed Straight Test Track</h2>
          <p className="mt-0.5 text-xs text-stone-600 leading-snug">
            Long, flat asphalt road with painted boundary lines & distance markers. Free-orbit camera follows vehicle.
          </p>

          <div className="mt-3 flex flex-wrap gap-1.5">
            <button
              onClick={onReset}
              className="rounded-lg bg-stone-900 px-3 py-1 text-xs font-medium text-stone-100 shadow hover:bg-stone-800 active:scale-95 transition-all"
            >
              Reset to Start [R]
            </button>
            <button
              onClick={onApplyPush}
              className="rounded-lg border border-amber-300 bg-amber-50 px-2.5 py-1 text-xs font-medium text-amber-900 shadow-sm hover:bg-amber-100 active:scale-95 transition-all"
            >
              Lateral Push [P]
            </button>
          </div>
        </div>

        {/* Center: Live Powertrain Tachometer & Shifter Cluster */}
        <div className="pointer-events-auto flex items-center gap-3 rounded-2xl border border-stone-300/85 bg-stone-50/95 px-4 py-2.5 shadow-lg backdrop-blur-md">
          {/* Gear Indicator Badge */}
          <div className="flex flex-col items-center">
            <span className="text-[9px] font-bold uppercase text-stone-400">Gear</span>
            <div className="relative flex items-center justify-center">
              <div
                className={`flex h-11 w-11 items-center justify-center rounded-xl font-mono text-xl font-black shadow-inner transition-colors ${
                  telemetry.currentGear === -1
                    ? 'bg-amber-600 text-white'
                    : telemetry.currentGear === 0
                    ? 'bg-stone-300 text-stone-700'
                    : 'bg-stone-900 text-emerald-400'
                }`}
              >
                {getGearLabel(telemetry.currentGear)}
              </div>
              {telemetry.isShifting && (
                <span className="absolute -bottom-2 whitespace-nowrap rounded bg-amber-500 px-1 text-[8px] font-bold text-white shadow animate-pulse">
                  SHIFT
                </span>
              )}
            </div>
            {/* Manual shift buttons */}
            <div className="mt-1 flex items-center gap-1">
              <button
                onClick={onShiftDown}
                title="Downshift [Q]"
                className="h-4 w-4 rounded bg-stone-200 text-[10px] font-bold text-stone-700 hover:bg-stone-300 active:scale-95"
              >
                -
              </button>
              <button
                onClick={onShiftUp}
                title="Upshift [E]"
                className="h-4 w-4 rounded bg-stone-200 text-[10px] font-bold text-stone-700 hover:bg-stone-300 active:scale-95"
              >
                +
              </button>
            </div>
          </div>

          {/* Tachometer & Speed */}
          <div className="flex flex-col justify-between min-w-[150px]">
            <div className="flex items-baseline justify-between">
              <div className="flex items-baseline gap-1">
                <span className={`font-mono text-2xl font-black ${isRedline ? 'text-red-600 animate-pulse' : 'text-stone-900'}`}>
                  {telemetry.engineRpm.toFixed(0)}
                </span>
                <span className="text-[10px] font-bold text-stone-400">RPM</span>
              </div>
              <div className="flex items-baseline gap-1">
                <span className="font-mono text-lg font-bold text-stone-900">
                  {telemetry.speedKmh.toFixed(1)}
                </span>
                <span className="text-[10px] font-semibold text-stone-500">km/h</span>
              </div>
            </div>

            {/* Graphical RPM Bar */}
            <div className="mt-1 h-2 w-full overflow-hidden rounded-full bg-stone-200">
              <div
                className={`h-full transition-all duration-75 ${
                  isRedline ? 'bg-red-600' : telemetry.engineRpm > 4800 ? 'bg-amber-500' : 'bg-emerald-500'
                }`}
                style={{ width: `${rpmProgress}%` }}
              />
            </div>

            <div className="mt-1 flex items-center justify-between text-[10px] text-stone-500 font-mono">
              <span>Torque: {telemetry.engineTorque.toFixed(0)} N·m</span>
              <span>Aero: -{telemetry.aeroDragN.toFixed(0)} N</span>
            </div>
          </div>

          {/* Pedals & ABS / Layout Status Badges */}
          <div className="flex flex-col gap-1.5 border-l border-stone-200 pl-3">
            {/* Throttle & Brake Bars */}
            <div className="flex items-center gap-1.5 text-[10px]">
              <span className="w-5 text-stone-500 font-bold">THR</span>
              <div className="h-1.5 w-14 overflow-hidden rounded-full bg-stone-200">
                <div
                  className="h-full bg-emerald-500 transition-all duration-75"
                  style={{ width: `${telemetry.throttlePct}%` }}
                />
              </div>
              <span className="w-6 font-mono text-[9px] text-stone-700">{telemetry.throttlePct.toFixed(0)}%</span>
            </div>

            <div className="flex items-center gap-1.5 text-[10px]">
              <span className="w-5 text-stone-500 font-bold">BRK</span>
              <div className="h-1.5 w-14 overflow-hidden rounded-full bg-stone-200">
                <div
                  className="h-full bg-red-500 transition-all duration-75"
                  style={{ width: `${telemetry.brakePct}%` }}
                />
              </div>
              <span className="w-6 font-mono text-[9px] text-stone-700">{telemetry.brakePct.toFixed(0)}%</span>
            </div>

            {/* Layout, Mode & ABS Buttons */}
            <div className="flex items-center gap-1 mt-0.5">
              <button
                onClick={onToggleLayout}
                title="Toggle Drivetrain Layout [G]"
                className={`rounded px-1.5 py-0.5 text-[10px] font-bold transition-all ${
                  telemetry.drivetrainLayout === 'AWD'
                    ? 'bg-emerald-600 text-white'
                    : telemetry.drivetrainLayout === 'RWD'
                    ? 'bg-blue-600 text-white'
                    : 'bg-purple-600 text-white'
                }`}
              >
                {telemetry.drivetrainLayout}
              </button>
              <button
                onClick={onToggleTransMode}
                title="Toggle Auto/Manual [M]"
                className="rounded bg-stone-200 px-1.5 py-0.5 text-[10px] font-bold text-stone-700 hover:bg-stone-300"
              >
                {telemetry.transmissionMode === 'auto' ? 'AUTO' : 'MANUAL'}
              </button>
              <button
                onClick={onToggleAbs}
                title="Toggle ABS [B]"
                className={`rounded px-1.5 py-0.5 text-[10px] font-bold transition-all ${
                  !telemetry.absEnabled
                    ? 'bg-stone-200 text-stone-400 line-through'
                    : telemetry.absGlobalActive
                    ? 'bg-amber-500 text-white animate-pulse'
                    : 'bg-stone-800 text-stone-100'
                }`}
              >
                ABS
              </button>
            </div>
          </div>
        </div>

        {/* Top Right Controls & Toggle */}
        <div className="pointer-events-auto flex items-center gap-2">
          {/* Active Steer Command Badge */}
          <div className="flex items-center gap-1.5 rounded-xl border border-stone-300/80 bg-stone-50/90 px-3 py-1.5 shadow-sm text-xs backdrop-blur-md">
            <span className="text-[11px] font-semibold text-stone-500">Steer Cmd:</span>
            <span
              className={`rounded px-2 py-0.5 font-bold font-mono text-xs ${
                telemetry.steerCmd === 'LEFT'
                  ? 'bg-blue-600 text-white animate-pulse'
                  : telemetry.steerCmd === 'RIGHT'
                  ? 'bg-orange-600 text-white animate-pulse'
                  : 'bg-stone-200 text-stone-600'
              }`}
            >
              {telemetry.steerCmd}
            </span>
          </div>

          <div className="hidden sm:flex items-center gap-1.5 rounded-xl border border-stone-300/80 bg-stone-50/90 px-3 py-1.5 shadow-sm text-xs backdrop-blur-md">
            <span
              className={`rounded px-1.5 py-0.5 font-bold ${
                telemetry.handbrakeActive ? 'bg-amber-600 text-white animate-pulse' : 'bg-stone-200 text-stone-500'
              }`}
            >
              Space: Handbrake
            </span>
          </div>

          <button
            onClick={onToggleVisible}
            className={`rounded-lg px-3 py-1.5 text-xs font-semibold shadow-md backdrop-blur-md transition-all ${
              visible
                ? 'border border-stone-400 bg-stone-900 text-stone-50'
                : 'border border-stone-300 bg-white/90 text-stone-700 hover:bg-stone-50'
            }`}
          >
            {visible ? 'Hide Telemetry [F3]' : 'Show Telemetry [F3]'}
          </button>
        </div>
      </div>

      {/* Main Telemetry Panel (F3) */}
      {visible && (
        <div className="pointer-events-auto mt-auto mb-2 flex flex-wrap items-end gap-3 max-h-[85vh] overflow-y-auto">
          {/* Chassis State & FIX 1b Kinematics */}
          <div className="w-76 rounded-xl border border-stone-300/80 bg-stone-50/95 p-3 shadow-lg backdrop-blur-md">
            <div className="flex items-center justify-between border-b border-stone-200 pb-1">
              <h3 className="text-xs font-bold uppercase tracking-wider text-stone-500">Chassis Kinematics</h3>
              <span className="text-[10px] font-bold text-emerald-700">Step 4 Powertrain</span>
            </div>

            <div className="mt-2 grid grid-cols-2 gap-1.5 text-xs">
              <div className="rounded-lg bg-white/80 p-1.5 border border-stone-200">
                <span className="text-[10px] text-stone-500">Speed</span>
                <div className="text-sm font-bold text-stone-900">
                  {telemetry.speedKmh.toFixed(1)} <span className="text-[10px] font-normal">km/h</span>
                </div>
              </div>
              <div className="rounded-lg bg-white/80 p-1.5 border border-stone-200">
                <span className="text-[10px] text-stone-500">Steer Cmd</span>
                <div
                  className={`text-sm font-mono font-bold ${
                    telemetry.steerCmd === 'LEFT'
                      ? 'text-blue-700'
                      : telemetry.steerCmd === 'RIGHT'
                      ? 'text-orange-700'
                      : 'text-stone-700'
                  }`}
                >
                  {telemetry.steerCmd}
                </div>
              </div>

              {/* Yaw rate (positive = turning left) */}
              <div className="rounded-lg bg-emerald-50/90 p-1.5 border border-emerald-300">
                <span className="text-[10px] font-semibold text-emerald-900">Yaw Rate (+Left)</span>
                <div className="text-sm font-mono font-bold text-emerald-950">
                  {telemetry.yawRateDeg > 0 ? '+' : ''}
                  {telemetry.yawRateDeg.toFixed(1)}°/s
                </div>
              </div>

              {/* Sideslip */}
              <div className="rounded-lg bg-emerald-50/90 p-1.5 border border-emerald-300">
                <span className="text-[10px] font-semibold text-emerald-900">Body Sideslip (β)</span>
                <div className="text-sm font-mono font-bold text-emerald-950">
                  {telemetry.bodySideslipDeg.toFixed(1)}°
                </div>
              </div>

              {/* Front Wheel Steer Angles (Positive = Left) */}
              <div className="rounded-lg bg-white/80 p-1.5 border border-stone-200">
                <span className="text-[10px] text-stone-500">FL Angle (+Left)</span>
                <div className="text-xs font-mono font-bold text-stone-800">
                  {telemetry.steerAngleFLDeg > 0 ? '+' : ''}
                  {telemetry.steerAngleFLDeg.toFixed(1)}°
                </div>
              </div>
              <div className="rounded-lg bg-white/80 p-1.5 border border-stone-200">
                <span className="text-[10px] text-stone-500">FR Angle (+Left)</span>
                <div className="text-xs font-mono font-bold text-stone-800">
                  {telemetry.steerAngleFRDeg > 0 ? '+' : ''}
                  {telemetry.steerAngleFRDeg.toFixed(1)}°
                </div>
              </div>
            </div>

            {/* Aerodynamic forces readout */}
            <div className="mt-2 rounded-lg bg-stone-100/80 p-1.5 text-[10px] border border-stone-200">
              <div className="flex justify-between text-stone-700">
                <span>Aero Drag (v²):</span>
                <b className="font-mono text-stone-900">-{telemetry.aeroDragN.toFixed(0)} N</b>
              </div>
              <div className="flex justify-between text-stone-700 mt-0.5">
                <span>Aero Downforce:</span>
                <b className="font-mono text-stone-900">{telemetry.aeroDownforceN.toFixed(0)} N</b>
              </div>
              <div className="flex justify-between text-stone-700 mt-0.5">
                <span>Clutch Lockup:</span>
                <b className="font-mono text-stone-900">{(telemetry.clutchEngagement * 100).toFixed(0)}%</b>
              </div>
            </div>

            {/* Live Slip vs Lateral Force Plot for Front-Left Wheel */}
            <div className="mt-2">
              <div className="flex items-center justify-between text-[10px] font-medium text-stone-600 mb-1">
                <span>FL Fy vs Slip Angle</span>
                <span className="text-orange-600 font-bold">● Live Op Point</span>
              </div>
              <canvas
                ref={canvasRef}
                width={250}
                height={75}
                className="w-full rounded-lg border border-stone-700 shadow-inner"
              />
              <div className="mt-0.5 flex justify-between text-[9px] text-stone-400">
                <span>-25° (Right)</span>
                <span>Slip Angle α</span>
                <span>+25° (Left)</span>
              </div>
            </div>
          </div>

          {/* 4-Wheel Detailed Dynamic Telemetry */}
          <div className="flex-1 min-w-[340px] max-w-3xl rounded-xl border border-stone-300/80 bg-stone-50/95 p-3 shadow-lg backdrop-blur-md">
            <div className="flex items-center justify-between border-b border-stone-200 pb-1.5">
              <h3 className="text-xs font-bold uppercase tracking-wider text-stone-500">
                4-Wheel Dynamic Telemetry & Friction Ellipse
              </h3>
              <div className="flex items-center gap-2 text-[10px] text-stone-500">
                <span className="inline-flex items-center gap-1">
                  <span className="inline-block h-2 w-2 rounded-full bg-red-500"></span> Fx (Long)
                </span>
                <span className="inline-flex items-center gap-1">
                  <span className="inline-block h-2 w-2 rounded-full bg-blue-500"></span> Fy (Lat)
                </span>
              </div>
            </div>

            <div className="mt-2 grid grid-cols-2 lg:grid-cols-4 gap-2">
              {telemetry.wheelsDyn.map((dyn, i) => {
                const susp = telemetry.wheelsSusp[i];
                const steerDeg = (dyn.steerAngle * 180) / Math.PI;
                const gripPct = Math.min(100, Math.round(dyn.gripUtilization * 100));
                const isAbsPulsing = telemetry.absActive[i];

                return (
                  <div
                    key={dyn.id}
                    className={`rounded-lg border p-2 transition-colors ${
                      isAbsPulsing
                        ? 'border-amber-400 bg-amber-50/80 animate-pulse'
                        : susp.grounded
                        ? 'border-stone-300 bg-white/90'
                        : 'border-sky-300 bg-sky-50/60'
                    }`}
                  >
                    <div className="flex items-center justify-between border-b border-stone-100 pb-1">
                      <div className="flex items-center gap-1">
                        <span className="font-bold text-xs text-stone-800">{dyn.id}</span>
                        {isAbsPulsing && (
                          <span className="rounded bg-amber-500 px-1 text-[8px] font-bold text-white">ABS</span>
                        )}
                      </div>
                      <span className="rounded bg-stone-200 px-1 py-0.2 text-[9px] font-mono uppercase text-stone-700">
                        {dyn.surface}
                      </span>
                      {i < 2 ? (
                        <span className="text-[10px] font-mono text-emerald-800 font-semibold">
                          δ: {steerDeg > 0 ? '+' : ''}{steerDeg.toFixed(1)}°
                        </span>
                      ) : (
                        <span className="text-[9px] text-stone-400">Rear</span>
                      )}
                    </div>

                    <div className="mt-1.5 grid grid-cols-2 gap-1 text-[10px] text-stone-600">
                      <div>
                        <span>Fz:</span> <b className="font-mono text-stone-900">{susp.normalLoad.toFixed(0)} N</b>
                      </div>
                      <div>
                        <span>RPM:</span> <b className="font-mono text-stone-900">{dyn.rpm.toFixed(0)}</b>
                      </div>
                    </div>

                    <div className="mt-1 grid grid-cols-2 gap-1 text-[10px] text-stone-600">
                      <div>
                        <span>Slip κ:</span>{' '}
                        <b
                          className={`font-mono ${
                            Math.abs(dyn.slipRatio) > 0.3 ? 'text-amber-600' : 'text-stone-800'
                          }`}
                        >
                          {dyn.slipRatio.toFixed(2)}
                        </b>
                      </div>
                      <div>
                        <span>Angle α:</span>{' '}
                        <b
                          className={`font-mono ${
                            Math.abs(dyn.slipAngleDeg) > 8 ? 'text-amber-600' : 'text-stone-800'
                          }`}
                        >
                          {dyn.slipAngleDeg.toFixed(1)}°
                        </b>
                      </div>
                    </div>

                    <div className="mt-1.5 rounded bg-stone-100/80 p-1 text-[10px] space-y-0.5 font-mono">
                      <div className="flex justify-between text-red-700">
                        <span>Fx (Long):</span>
                        <b>{dyn.Fx.toFixed(0)} N</b>
                      </div>
                      <div className="flex justify-between text-blue-700">
                        <span>Fy (Lat):</span>
                        <b>{dyn.Fy.toFixed(0)} N</b>
                      </div>
                    </div>

                    <div className="mt-1.5">
                      <div className="flex justify-between text-[9px] text-stone-600">
                        <span>Ellipse Usage:</span>
                        <b className={gripPct >= 99 ? 'text-red-600 font-bold' : 'text-stone-700'}>
                          {gripPct}%
                        </b>
                      </div>
                      <div className="mt-0.5 h-1.5 w-full overflow-hidden rounded-full bg-stone-200">
                        <div
                          className={`h-full rounded-full transition-all duration-75 ${
                            gripPct >= 95 ? 'bg-red-500' : gripPct >= 75 ? 'bg-amber-500' : 'bg-emerald-500'
                          }`}
                          style={{ width: `${gripPct}%` }}
                        />
                      </div>
                    </div>
                  </div>
                );
              })}
            </div>

            <div className="mt-2.5 border-t border-stone-200 pt-2 flex flex-wrap items-center justify-between gap-1 text-[10px]">
              <div className="flex items-center gap-1.5">
                <span className="font-semibold text-stone-500">Track Quick Actions:</span>
                <button
                  onClick={onReset}
                  className="rounded bg-stone-900 px-2 py-0.5 font-medium text-stone-100 hover:bg-stone-800 shadow-sm"
                >
                  Reset Start [R]
                </button>
                <button
                  onClick={onToggleLayout}
                  className="rounded bg-stone-200 px-2 py-0.5 font-medium text-stone-700 hover:bg-stone-300"
                >
                  Toggle Layout [G]
                </button>
                <button
                  onClick={onToggleTransMode}
                  className="rounded bg-stone-200 px-2 py-0.5 font-medium text-stone-700 hover:bg-stone-300"
                >
                  Mode [M]
                </button>
                <button
                  onClick={onToggleAbs}
                  className="rounded bg-stone-200 px-2 py-0.5 font-medium text-stone-700 hover:bg-stone-300"
                >
                  ABS [B]
                </button>
              </div>
              <span className="text-stone-400">Mouse: Left-Drag to Orbit • Scroll to Zoom</span>
            </div>
          </div>
        </div>
      )}

      {/* Bottom Hint Bar */}
      <div className="flex items-center justify-between text-xs text-stone-500">
        <div className="flex flex-wrap items-center gap-2">
          <span className="rounded bg-stone-200/80 px-2 py-0.5 font-mono text-[11px] text-stone-800">
            [W] Throttle
          </span>
          <span className="rounded bg-stone-200/80 px-2 py-0.5 font-mono text-[11px] text-stone-800">
            [S] Brake / Rev
          </span>
          <span className="rounded bg-stone-200/80 px-2 py-0.5 font-mono text-[11px] text-stone-800">
            [A/D] Steer
          </span>
          <span className="rounded bg-stone-200/80 px-2 py-0.5 font-mono text-[11px] text-stone-800">
            [Space] Handbrake
          </span>
          <span className="rounded bg-stone-200/80 px-2 py-0.5 font-mono text-[11px] text-stone-800">
            [Q/E] Shift Down/Up
          </span>
          <span className="rounded bg-stone-200/80 px-2 py-0.5 font-mono text-[11px] text-stone-800">
            [M] Auto/Manual
          </span>
          <span className="rounded bg-stone-200/80 px-2 py-0.5 font-mono text-[11px] text-stone-800">
            [G] AWD/RWD/FWD
          </span>
          <span className="rounded bg-stone-200/80 px-2 py-0.5 font-mono text-[11px] text-stone-800">
            [B] ABS On/Off
          </span>
          <span className="rounded bg-stone-200/80 px-2 py-0.5 font-mono text-[11px] text-stone-800">
            [R] Reset
          </span>
          <span className="rounded bg-stone-200/80 px-2 py-0.5 font-mono text-[11px] text-stone-800">
            [F3] HUD
          </span>
          <span className="rounded bg-stone-100 px-2 py-0.5 font-mono text-[11px] text-stone-600 border border-stone-200">
            Mouse Drag: Free Orbit & Zoom
          </span>
        </div>
        <div className="text-[11px] text-stone-400">Step 4A • Straight Track & Free-Orbit Follow Camera</div>
      </div>
    </div>
  );
};
