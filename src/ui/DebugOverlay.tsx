/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import React from 'react';

export interface TelemetryData {
  speedKmh?: number;
  engineRpm?: number;
  [key: string]: unknown;
}

export interface DebugOverlayProps {
  visible?: boolean;
  [key: string]: unknown;
}

/**
 * Stage A8: Telemetry HUD & Debug UI removed for clean interface.
 * Returns null to eliminate all debug text, graphs, and HUD overlays.
 */
export const DebugOverlay: React.FC<DebugOverlayProps> = () => {
  return null;
};
