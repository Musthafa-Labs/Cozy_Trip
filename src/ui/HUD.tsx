/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import React from 'react';

export interface HUDProps {
  speedKmh?: number;
  currentGear?: number;
  mode?: 'auto' | 'manual';
  rangeMode?: 'high' | 'low';
}

/**
 * Minimal Driving HUD Overlay.
 * Displays strictly Current Gear, Vehicle Speed (km/h), Transmission Mode, and Transfer Case Range.
 * Zero telemetry graphs, frame timers, or debug clutter.
 */
export const HUD: React.FC<HUDProps> = ({
  speedKmh = 0,
  currentGear = 1,
  mode = 'auto',
  rangeMode = 'high',
}) => {
  // Format Gear: -1 -> 'R', 0 -> 'N', 1..6 -> '1'..'6'
  let gearText = '1';
  if (currentGear === -1) {
    gearText = 'R';
  } else if (currentGear === 0) {
    gearText = 'N';
  } else {
    gearText = String(Math.max(1, Math.min(6, currentGear)));
  }

  const speedDisplay = Math.max(0, Math.round(speedKmh));
  const isAuto = mode === 'auto';
  const isHighRange = rangeMode === 'high';

  return (
    <div
      data-testid="driving-hud"
      className="pointer-events-none fixed bottom-6 right-6 z-40 select-none font-mono"
    >
      <div className="flex items-center gap-4 rounded-2xl border border-stone-700/60 bg-stone-900/85 px-4 py-3 shadow-2xl backdrop-blur-md text-stone-100">
        {/* Speed Readout */}
        <div className="flex flex-col items-start min-w-[70px]">
          <span className="text-3xl font-extrabold tracking-tight tabular-nums text-white">
            {speedDisplay}
          </span>
          <span className="text-[10px] font-bold tracking-widest text-stone-400 uppercase">
            KM/H
          </span>
        </div>

        {/* Divider */}
        <div className="h-9 w-px bg-stone-700/60" />

        {/* Gear Readout */}
        <div className="flex flex-col items-center min-w-[36px]">
          <span
            className={`text-3xl font-black ${
              gearText === 'R'
                ? 'text-rose-400'
                : gearText === 'N'
                ? 'text-amber-300'
                : 'text-amber-400'
            }`}
          >
            {gearText}
          </span>
          <span className="text-[10px] font-bold tracking-wider text-stone-400 uppercase">
            GEAR
          </span>
        </div>

        {/* Divider */}
        <div className="h-9 w-px bg-stone-700/60" />

        {/* Transmission Mode & Transfer Case Range Badges */}
        <div className="flex flex-col gap-1.5 min-w-[65px]">
          {/* Transmission Mode */}
          <span
            className={`px-2 py-0.5 rounded text-[10px] font-bold tracking-wider uppercase text-center border ${
              isAuto
                ? 'bg-emerald-950/70 border-emerald-500/40 text-emerald-300'
                : 'bg-amber-950/70 border-amber-500/40 text-amber-300'
            }`}
          >
            {isAuto ? 'AUTO' : 'MANUAL'}
          </span>

          {/* Transfer Case Range */}
          <span
            className={`px-2 py-0.5 rounded text-[10px] font-bold tracking-wider uppercase text-center border ${
              isHighRange
                ? 'bg-sky-950/70 border-sky-500/40 text-sky-300'
                : 'bg-orange-950/70 border-orange-500/40 text-orange-300'
            }`}
          >
            {isHighRange ? 'HIGH' : 'LOW'}
          </span>
        </div>
      </div>
    </div>
  );
};
