/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import * as THREE from 'three';
import { SurfaceType } from '../vehicle/tire';

export interface GroundQuery {
  height: number;
  normal: THREE.Vector3;
  surface: SurfaceType;
}

export interface ObstacleContactResult {
  penetration: number;
  normal: THREE.Vector3;
  surface: SurfaceType;
  pointOnObstacle: THREE.Vector3;
}

export interface TestStation {
  id: string;
  name: string;
  zone: string;
  description: string;
  position: THREE.Vector3;
  headingRad: number;
}

export interface StaticBoxObstacle {
  id: string;
  name: string;
  center: THREE.Vector3;
  halfExtents: THREE.Vector3; // halfWidth (X), halfHeight (Y), halfLength (Z)
  yawRad: number;
  surface: SurfaceType;
}

/**
 * Standard Hermite smoothstep: S(t) = t * t * (3 - 2 * t) clamped to 0..1
 */
function smoothstep(edge0: number, edge1: number, x: number): number {
  const t = Math.max(0, Math.min(1, (x - edge0) / (edge1 - edge0)));
  return t * t * (3 - 2 * t);
}

export class TestGround {
  public readonly mesh: THREE.Group;

  // Preallocated scratch vector and query object to ensure zero allocations per frame
  private readonly normalScratch = new THREE.Vector3(0, 1, 0);
  private readonly queryResult: GroundQuery = {
    height: 0.0,
    normal: this.normalScratch,
    surface: 'asphalt',
  };

  // Exact 1.2m x 1.2m x 1.2m static solid cube positioned directly in driving path at Z = 65m
  public readonly staticObstacles: StaticBoxObstacle[] = [
    {
      id: 'target_cube',
      name: 'Solid 1.2m Cube',
      center: new THREE.Vector3(0, 0.60, 65.0),
      halfExtents: new THREE.Vector3(0.60, 0.60, 0.60),
      yawRad: 0,
      surface: 'asphalt',
    },
  ];

  // Test stations: Cube collision, Ramp Jump, Ditches, Parking Hill, Side Slopes, Washboard, Moguls, Logs, and Articulation
  public readonly stations: TestStation[] = [
    {
      id: 'cube_collision_test',
      name: 'Solid Cube Collision Test',
      zone: 'Testing Arena',
      description: 'Single straight road with one solid 1.2m cube at Z = 65m. Approach and impact at 10, 30 or 50 km/h.',
      position: new THREE.Vector3(0, 1.06, 0),
      headingRad: 0,
    },
    {
      id: 'ramp_jump_test',
      name: 'Ramp Jump Test',
      zone: 'Zone 1: Ramp Jump',
      description: 'Ramp jump test at Z = 150..169.5. Approach at 30-70 km/h. Rises to 1.6m launch lip with flat landing.',
      position: new THREE.Vector3(0, 1.06, 100),
      headingRad: 0,
    },
    {
      id: 'ditch_test',
      name: 'Ditches & Obstacles Test',
      zone: 'Zone 2: Ditches',
      description: 'Trenches, step-downs, and 1m drop-off at Z = 300..400. Approach at 15-50 km/h.',
      position: new THREE.Vector3(0, 1.06, 280),
      headingRad: 0,
    },
    {
      id: 'parking_hill_test',
      name: 'Parking Hill Test (20° Incline)',
      zone: 'Zone 3: Parking Hill',
      description: '20° slope incline (36.4% grade, 3.64m rise) at Z = 420..430, plateau at Z = 430..445, descent at Z = 445..455.',
      position: new THREE.Vector3(0, 1.06, 405),
      headingRad: 0,
    },
    {
      id: 'side_slope_test',
      name: 'Side Slope Test (15° & 25° Banks)',
      zone: 'Zone 4: Side Slopes',
      description: 'Lateral banking tracks: 15° side slope at Z = 465..485 and 25° side slope at Z = 495..515 for X in [1, 6].',
      position: new THREE.Vector3(0, 1.06, 458),
      headingRad: 0,
    },
    {
      id: 'washboard_test',
      name: 'Washboard Bumps Test',
      zone: 'Zone 5: Washboard',
      description: 'Sinusoidal high-frequency ripples (1.2m wavelength, 0.06m amplitude) at Z = 540..580. Test suspension chatter at 20-50 km/h.',
      position: new THREE.Vector3(0, 1.06, 530),
      headingRad: 0,
    },
    {
      id: 'moguls_test',
      name: 'Moguls & Articulation Test',
      zone: 'Zone 6: Moguls',
      description: 'Staggered 2D sine mounds (4.0m wavelength, 180° left/right phase shift, 0.15m amplitude) at Z = 590..640. Test cross-axle articulation at 10-25 km/h.',
      position: new THREE.Vector3(0, 1.06, 580),
      headingRad: 0,
    },
    {
      id: 'logs_test',
      name: 'Log Obstacles Crawl Test',
      zone: 'Zone 7: Log Obstacles',
      description: '4 transverse semi-cylindrical logs (radii 0.19-0.24m) at Z = 668, 676, 684, 692. Crawl at 5-15 km/h.',
      position: new THREE.Vector3(0, 1.06, 650),
      headingRad: 0,
    },
    {
      id: 'articulation_test',
      name: 'Rock Boulder Articulation Test',
      zone: 'Zone 8: Rock Articulation',
      description: 'Asymmetrical 3D rock mounds (heights 0.22-0.34m) alternating left and right at Z = 710..760. Test maximum cross-axle articulation.',
      position: new THREE.Vector3(0, 1.06, 700),
      headingRad: 0,
    },
  ];

  // Road geometry dimensions: 14m wide, extended to Z = 780m (-20m to +780m)
  public static readonly ROAD_WIDTH = 14.0;
  public static readonly ROAD_HALF_WIDTH = 7.0;
  public static readonly ROAD_START_Z = -20.0;
  public static readonly ROAD_END_Z = 780.0;

  // Pre-configured rock mounds for Zone 8 (Cross-Axle Articulation Course)
  private static readonly ROCKS: ReadonlyArray<{
    x: number;
    z: number;
    rx: number;
    rz: number;
    h: number;
  }> = [
    { x: -2.0, z: 715.0, rx: 1.4, rz: 1.6, h: 0.26 }, // Rock 1: Left track
    { x: 2.2, z: 721.0, rx: 1.5, rz: 1.7, h: 0.32 },  // Rock 2: Right track
    { x: -1.8, z: 727.0, rx: 1.3, rz: 1.5, h: 0.28 }, // Rock 3: Left track
    { x: 2.0, z: 733.0, rx: 1.6, rz: 1.8, h: 0.34 },  // Rock 4: Right track (heavy compression)
    { x: -2.2, z: 739.0, rx: 1.5, rz: 1.6, h: 0.30 }, // Rock 5: Left track
    { x: 1.8, z: 745.0, rx: 1.4, rz: 1.6, h: 0.27 },  // Rock 6: Right track
    { x: -1.9, z: 751.0, rx: 1.5, rz: 1.5, h: 0.25 }, // Rock 7: Left track
    { x: 2.1, z: 757.0, rx: 1.3, rz: 1.4, h: 0.22 },  // Rock 8: Right track
  ];

  // Pre-configured transverse logs for Zone 7 (Log Crawl Course)
  private static readonly LOGS: ReadonlyArray<{
    z: number;
    r: number;
  }> = [
    { z: 668.0, r: 0.20 }, // Log 1: 0.20m radius
    { z: 676.0, r: 0.24 }, // Log 2: 0.24m radius
    { z: 684.0, r: 0.19 }, // Log 3: 0.19m radius
    { z: 692.0, r: 0.22 }, // Log 4: 0.22m radius
  ];

  constructor() {
    this.mesh = new THREE.Group();
    this.buildScene();
  }

  /**
   * Smooth analytic terrain height function using smoothstep transitions only.
   * Zero vertical faces; C1 continuity everywhere.
   *
   * ZONE 1 (Ramp Jump):
   * - Full effect for |x| <= 4m, blending to 0 over the next 1m (|x| = 4..5m)
   * - Approach ramp: rises 0.6m, smoothstep over z = 150..162
   * - Launch ramp: rises a further 1.0m, smoothstep over z = 162..168 (height 1.6m at z = 168)
   * - Lip fall-away: height falls back to 0 with smoothstep over z = 168..169.5
   * - Flat road afterward (the landing area)
   *
   * ZONE 2 (Ditches):
   * - Full effect for |x| <= 7m, blending to 0 over 1m (|x| = 7..8m)
   * - Shallow trench: 0.2m deep between z = 300..304, smoothstep edges 1.2m (z = 298.8..305.2)
   * - Medium trench: 0.35m deep between z = 320..326, smoothstep edges 1.6m (z = 318.4..327.6)
   * - Two step-downs: -0.3m smoothstep over z = 340..341 and further -0.3m over z = 350..351,
   *   then smooth recovery back to 0 with smoothstep over z = 365..369
   * - Drop-off: -1.0m with smoothstep over z = 380..381.5,
   *   then smooth ramp back up to 0 with smoothstep over z = 392..400
   *
   * ZONE 3 (Parking Hill):
   * - Full effect for |x| <= 6m, blending to 0 over 1m (|x| = 6..7m)
   * - Approach apron: flat at z = 410..420
   * - 20° incline: rises to 3.64m over z = 420..430 with smoothstep entry (420..422) and exit (428..430)
   * - Plateau: holds constant height of 3.64m over z = 430..445
   * - Descent ramp: smoothstep back to 0m over z = 445..455
   *
   * ZONE 4 (Side Slopes):
   * - 15° side slope (z = 465..485): rises linearly by +0.268 m/m for x in [1, 6], smoothstep entry/exit along Z
   * - 25° side slope (z = 495..515): rises linearly by +0.466 m/m for x in [1, 6], smoothstep entry/exit along Z
   *
   * ZONE 5 (Washboard Bumps):
   * - Washboard track (z = 540..580): regular sinusoidal corrugated ripples with wavelength 1.2m and amplitude 0.06m
   * - Smoothstep entry (z = 540..543) and exit (z = 577..580) envelope blending
   * - Full effect for |x| <= 6m, blending to 0 over 1m (|x| = 6..7m)
   *
   * ZONE 6 (Moguls & Staggered Mounds):
   * - Mogul field (z = 590..640): staggered left/right mounds using 2D sine math: z-wavelength 4.0m,
   *   x-phase shift of 180° between left (x < 0) and right (x > 0) tracks, peak amplitude +0.15m
   * - Smoothstep entry (z = 590..594) and exit (z = 636..640) envelope blending
   * - Full effect for |x| <= 6m, blending to 0 over 1m (|x| = 6..7m)
   *
   * ZONE 7 (Log Obstacles):
   * - Log crawl track (z = 660..700): 4 fallen logs lying transversely across track at z = 668, 676, 684, 692
   * - Semi-cylindrical ridges of radius 0.19m - 0.24m with smoothstep base contact
   * - Full effect for |x| <= 6m, blending to 0 over 1m (|x| = 6..7m)
   *
   * ZONE 8 (Rock Boulder Articulation Course):
   * - Asymmetrical 3D rock mounds (z = 710..760): varying heights 0.22m - 0.34m, radii 1.3m - 1.8m
   * - Staggered diagonally across left and right tracks to test maximum cross-axle articulation
   * - Full effect for |x| <= 6m, blending to 0 over 1m (|x| = 6..7m)
   */
  public terrainHeight(x: number, z: number): number {
    const absX = Math.abs(x);
    let h1 = 0.0;
    let h2 = 0.0;
    let h3 = 0.0;
    let h4 = 0.0;
    let h5 = 0.0;
    let h6 = 0.0;
    let h7 = 0.0;
    let h8 = 0.0;

    // ZONE 1: Ramp Jump (Z = 150..169.5)
    if (absX < 5.0 && z >= 150.0 && z <= 169.5) {
      let rawH = 0.0;
      if (z <= 162.0) {
        rawH = 0.6 * smoothstep(150.0, 162.0, z);
      } else if (z <= 168.0) {
        rawH = 0.6 + 1.0 * smoothstep(162.0, 168.0, z);
      } else {
        rawH = 1.6 * (1.0 - smoothstep(168.0, 169.5, z));
      }

      const lateralBlend = absX <= 4.0 ? 1.0 : (1.0 - smoothstep(4.0, 5.0, absX));
      h1 = rawH * lateralBlend;
    }

    // ZONE 2: Ditches (Z = 298.8..400.0)
    if (absX < 8.0 && z >= 298.8 && z <= 400.0) {
      let rawH = 0.0;

      if (z <= 305.2) {
        // Shallow trench: 0.2m deep between z = 300..304, smoothstep edges 1.2m
        if (z < 300.0) {
          rawH = -0.2 * smoothstep(298.8, 300.0, z);
        } else if (z <= 304.0) {
          rawH = -0.2;
        } else {
          rawH = -0.2 * (1.0 - smoothstep(304.0, 305.2, z));
        }
      } else if (z >= 318.4 && z <= 327.6) {
        // Medium trench: 0.35m deep between z = 320..326, smoothstep edges 1.6m
        if (z < 320.0) {
          rawH = -0.35 * smoothstep(318.4, 320.0, z);
        } else if (z <= 326.0) {
          rawH = -0.35;
        } else {
          rawH = -0.35 * (1.0 - smoothstep(326.0, 327.6, z));
        }
      } else if (z >= 340.0 && z <= 369.0) {
        // Two step-downs: -0.3m smoothstep over z = 340..341 and further -0.3m over z = 350..351,
        // then smooth recovery back to 0 with smoothstep over z = 365..369
        if (z < 341.0) {
          rawH = -0.3 * smoothstep(340.0, 341.0, z);
        } else if (z < 350.0) {
          rawH = -0.3;
        } else if (z < 351.0) {
          rawH = -0.3 - 0.3 * smoothstep(350.0, 351.0, z);
        } else if (z < 365.0) {
          rawH = -0.6;
        } else {
          rawH = -0.6 * (1.0 - smoothstep(365.0, 369.0, z));
        }
      } else if (z >= 380.0 && z <= 400.0) {
        // Drop-off: -1.0m with smoothstep over z = 380..381.5,
        // then smooth ramp back up to 0 with smoothstep over z = 392..400
        if (z < 381.5) {
          rawH = -1.0 * smoothstep(380.0, 381.5, z);
        } else if (z < 392.0) {
          rawH = -1.0;
        } else {
          rawH = -1.0 * (1.0 - smoothstep(392.0, 400.0, z));
        }
      }

      const lateralBlend = absX <= 7.0 ? 1.0 : (1.0 - smoothstep(7.0, 8.0, absX));
      h2 = rawH * lateralBlend;
    }

    // ZONE 3: Parking Hill (20° incline, 3.64m rise, Z = 420.0..455.0)
    if (absX < 7.0 && z >= 420.0 && z <= 455.0) {
      let rawH = 0.0;
      if (z < 430.0) {
        // 20-Degree Incline: rises to 3.64m over z = 420..430 with smoothstep entry (420..422) and exit (428..430)
        const m = 3.64 / 8.0; // 0.455 m/m
        if (z < 422.0) {
          const u = (z - 420.0) / 2.0;
          rawH = 2.0 * m * (u * u * u - 0.5 * u * u * u * u);
        } else if (z <= 428.0) {
          rawH = m + m * (z - 422.0);
        } else {
          const u = (z - 428.0) / 2.0;
          rawH = 7.0 * m + 2.0 * m * (u - (u * u * u - 0.5 * u * u * u * u));
        }
      } else if (z <= 445.0) {
        // Plateau: holds constant height of 3.64m over z = 430..445
        rawH = 3.64;
      } else {
        // Descent ramp: smoothstep back to 0m over z = 445..455
        rawH = 3.64 * (1.0 - smoothstep(445.0, 455.0, z));
      }

      const lateralBlend = absX <= 6.0 ? 1.0 : (1.0 - smoothstep(6.0, 7.0, absX));
      h3 = rawH * lateralBlend;
    }

    // ZONE 4: Side Slopes (Banking test tracks at Z = 465..485 and Z = 495..515)
    if (x > 0.0 && x < 8.0 && ((z >= 465.0 && z <= 485.0) || (z >= 495.0 && z <= 515.0))) {
      let slope = 0.0;
      let zBlend = 0.0;

      if (z <= 485.0) {
        // 15-Degree Side Slope (15° roll angle, slope = 0.268 m/m)
        slope = 0.268;
        if (z < 467.0) {
          zBlend = smoothstep(465.0, 467.0, z);
        } else if (z <= 483.0) {
          zBlend = 1.0;
        } else {
          zBlend = 1.0 - smoothstep(483.0, 485.0, z);
        }
      } else {
        // 25-Degree Side Slope (25° roll angle, slope = 0.466 m/m)
        slope = 0.466;
        if (z < 497.0) {
          zBlend = smoothstep(495.0, 497.0, z);
        } else if (z <= 513.0) {
          zBlend = 1.0;
        } else {
          zBlend = 1.0 - smoothstep(513.0, 515.0, z);
        }
      }

      // Lateral banking profile across X:
      // x in [0, 1]: smoothstep transition into bank
      // x in [1, 6]: strictly linear slope (+0.268 or +0.466 m/m)
      // x in [6, 7]: smoothstep transition leveling off at road edge
      // x in [7, 8]: smoothstep blend down to 0 on dirt shoulder
      let rawLatH = 0.0;
      if (x < 1.0) {
        rawLatH = slope * (x * x * x - 0.5 * x * x * x * x);
      } else if (x <= 6.0) {
        rawLatH = 0.5 * slope + slope * (x - 1.0);
      } else if (x <= 7.0) {
        const u = x - 6.0;
        rawLatH = 5.5 * slope + slope * (u - (u * u * u - 0.5 * u * u * u * u));
      } else {
        const hEdge = 6.0 * slope;
        rawLatH = hEdge * (1.0 - smoothstep(7.0, 8.0, x));
      }

      h4 = rawLatH * zBlend;
    }

    // ZONE 5: Washboard Bumps (Z = 540.0..580.0, 1.2m wavelength, 0.06m amplitude)
    if (absX < 7.0 && z >= 540.0 && z <= 580.0) {
      let zEnvelope = 0.0;
      if (z < 543.0) {
        zEnvelope = smoothstep(540.0, 543.0, z);
      } else if (z <= 577.0) {
        zEnvelope = 1.0;
      } else {
        zEnvelope = 1.0 - smoothstep(577.0, 580.0, z);
      }

      const wave = 0.06 * Math.sin((2.0 * Math.PI * (z - 540.0)) / 1.2);
      const lateralBlend = absX <= 6.0 ? 1.0 : (1.0 - smoothstep(6.0, 7.0, absX));
      h5 = wave * zEnvelope * lateralBlend;
    }

    // ZONE 6: Moguls & Staggered Mounds (Z = 590.0..640.0, 4.0m wavelength, 0.15m amplitude, 180° L/R phase shift)
    if (absX < 7.0 && z >= 590.0 && z <= 640.0) {
      let zEnvelope = 0.0;
      if (z < 594.0) {
        zEnvelope = smoothstep(590.0, 594.0, z);
      } else if (z <= 636.0) {
        zEnvelope = 1.0;
      } else {
        zEnvelope = 1.0 - smoothstep(636.0, 640.0, z);
      }

      // 2D sine math: z-wavelength 4.0m, x-phase shift of 180° between left (x < 0) and right (x > 0) tracks
      const clampedX = Math.max(-6.0, Math.min(6.0, x));
      const xWave = Math.sin((Math.PI * clampedX) / 6.0);
      const zWave = Math.sin((2.0 * Math.PI * (z - 590.0)) / 4.0);
      const mound = 0.15 * xWave * zWave;

      const lateralBlend = absX <= 6.0 ? 1.0 : (1.0 - smoothstep(6.0, 7.0, absX));
      h6 = mound * zEnvelope * lateralBlend;
    }

    // ZONE 7: Log Obstacles (Z = 660.0..700.0, 4 transverse semi-cylindrical logs)
    if (absX < 7.0 && z >= 660.0 && z <= 700.0) {
      let rawLogH = 0.0;
      for (let i = 0; i < TestGround.LOGS.length; i++) {
        const log = TestGround.LOGS[i];
        const dz = Math.abs(z - log.z);
        if (dz < log.r) {
          const u = dz / log.r; // 0 at log top, 1 at ground contact
          // Semi-cylinder with smooth C1 base contact: sqrt(1 - u^2) * (1 - u^6)
          // Preserves cylindrical crest for wheel contact while smoothly blending to 0 slope at base
          const u2 = u * u;
          const u6 = u2 * u2 * u2;
          const logProfile = log.r * Math.sqrt(Math.max(0, 1.0 - u2)) * (1.0 - u6);
          if (logProfile > rawLogH) {
            rawLogH = logProfile;
          }
        }
      }

      const lateralBlend = absX <= 6.0 ? 1.0 : (1.0 - smoothstep(6.0, 7.0, absX));
      h7 = rawLogH * lateralBlend;
    }

    // ZONE 8: Rocks & Wheel-Articulation Course (Z = 710.0..760.0, asymmetrical 3D rock mounds)
    if (absX < 7.0 && z >= 710.0 && z <= 762.0) {
      let rawRockH = 0.0;
      for (let i = 0; i < TestGround.ROCKS.length; i++) {
        const rock = TestGround.ROCKS[i];
        const dx = (x - rock.x) / rock.rx;
        const dz = (z - rock.z) / rock.rz;
        const distSq = dx * dx + dz * dz;
        if (distSq < 1.0) {
          // Smooth 3D dome: h * (1 - distSq)^2 ensures C1 smoothness and 0 slope at edge
          const moundH = rock.h * (1.0 - distSq) * (1.0 - distSq);
          if (moundH > rawRockH) {
            rawRockH = moundH;
          }
        }
      }

      const lateralBlend = absX <= 6.0 ? 1.0 : (1.0 - smoothstep(6.0, 7.0, absX));
      h8 = rawRockH * lateralBlend;
    }

    return h1 + h2 + h3 + h4 + h5 + h6 + h7 + h8;
  }

  /**
   * Fast, exact analytical ground query for road and terrain.
   * Central differences (epsilon = 0.05m) computes outward normal into preallocated scratch vector.
   * Surface is 'asphalt' for |x| <= 7 within road Z bounds, and 'dirt' beyond.
   */
  public getHeightNormalSurface(x: number, z: number): GroundQuery {
    const hCenter = this.terrainHeight(x, z);

    // If completely outside feature areas, ground is perfectly level (0, 1, 0)
    const absX = Math.abs(x);
    if (
      absX >= 8.1 ||
      (z < 149.0 && z > -25.0) ||
      (z > 170.5 && z < 297.8) ||
      (z > 401.0 && z < 419.0) ||
      (z > 456.0 && z < 464.0) ||
      (z > 486.0 && z < 494.0) ||
      (z > 516.0 && z < 539.0) ||
      (z > 581.0 && z < 589.0) ||
      (z > 641.0 && z < 659.0) ||
      (z > 701.0 && z < 709.0) ||
      z > 763.0
    ) {
      this.normalScratch.set(0, 1, 0);
    } else {
      const eps = 0.05;
      const hL = this.terrainHeight(x - eps, z);
      const hR = this.terrainHeight(x + eps, z);
      const hD = this.terrainHeight(x, z - eps);
      const hU = this.terrainHeight(x, z + eps);

      const dhdx = (hR - hL) / (2.0 * eps);
      const dhdz = (hU - hD) / (2.0 * eps);

      const invLen = 1.0 / Math.sqrt(dhdx * dhdx + 1.0 + dhdz * dhdz);
      this.normalScratch.set(-dhdx * invLen, invLen, -dhdz * invLen);
    }

    const onRoad =
      absX <= TestGround.ROAD_HALF_WIDTH &&
      z >= TestGround.ROAD_START_Z &&
      z <= TestGround.ROAD_END_Z;

    this.queryResult.height = hCenter;
    this.queryResult.surface = onRoad ? 'asphalt' : 'dirt';
    return this.queryResult;
  }

  /**
   * Evaluates exact 3D contact penetration and outward face normal against the solid cube.
   * Outputs the outward unit normal and penetration depth along the closest face.
   */
  public getObstacleContact(
    worldPoint: THREE.Vector3,
    outResult: ObstacleContactResult
  ): boolean {
    const obs = this.staticObstacles[0];

    // Distance to obstacle center
    const dx = worldPoint.x - obs.center.x;
    const dy = worldPoint.y - obs.center.y;
    const dz = worldPoint.z - obs.center.z;

    // Check against box bounds (with rotation if any)
    const cosY = Math.cos(-obs.yawRad);
    const sinY = Math.sin(-obs.yawRad);
    const localX = cosY * dx - sinY * dz;
    const localZ = sinY * dx + cosY * dz;
    const localY = dy;

    const distX = obs.halfExtents.x - Math.abs(localX);
    const distY = obs.halfExtents.y - Math.abs(localY);
    const distZ = obs.halfExtents.z - Math.abs(localZ);

    // If point is strictly inside the 3D cube volume
    if (distX > 0 && distY > 0 && distZ > 0) {
      // Find the closest face (smallest penetration distance)
      let minPen = distX;
      let localNormX = Math.sign(localX);
      let localNormY = 0;
      let localNormZ = 0;

      if (distZ < minPen) {
        minPen = distZ;
        localNormX = 0;
        localNormY = 0;
        localNormZ = Math.sign(localZ);
      }

      if (distY < minPen) {
        minPen = distY;
        localNormX = 0;
        localNormY = Math.sign(localY);
        localNormZ = 0;
      }

      // Transform outward local normal back to world space
      const cosYaw = Math.cos(obs.yawRad);
      const sinYaw = Math.sin(obs.yawRad);
      const worldNormX = cosYaw * localNormX - sinYaw * localNormZ;
      const worldNormZ = sinYaw * localNormX + cosYaw * localNormZ;

      outResult.penetration = minPen;
      outResult.normal.set(worldNormX, localNormY, worldNormZ).normalize();
      outResult.surface = obs.surface;
      outResult.pointOnObstacle.copy(worldPoint).addScaledVector(outResult.normal, minPen);
      return true;
    }

    return false;
  }

  /**
   * Constructs visual meshes:
   * 1. Road mesh evaluated directly from terrainHeight(x, z) at ~0.20m-0.5m grid spacing up to Z = 780m
   * 2. Edge markings and dashed center line following the 3D terrain profile (+0.003m offset)
   * 3. Surrounding ground mesh following terrain shoulders and blending to level ground (-0.015m offset)
   * 4. Roadside orientation signs at X = 9m for all test zones
   * 5. Solid 1.2m cube at (0, 0.6, 65)
   */
  private buildScene(): void {
    // =========================================================================
    // 1. Grid Definition along X and Z
    // =========================================================================
    // Road X coordinates: 29 points spanning -7.0m to +7.0m (0.5m spacing)
    // Ensures exact placement at key lateral blend boundaries: -7, -6, -5, -4, 0, 1, 4, 5, 6, 7
    const roadXCoords: number[] = [];
    for (let i = 0; i <= 28; i++) {
      roadXCoords.push(-TestGround.ROAD_HALF_WIDTH + (i / 28) * TestGround.ROAD_WIDTH);
    }

    // Road Z coordinates: dense longitudinal resolution across all active zones
    const zCoords: number[] = [];
    for (let z = -20.0; z < 140.0; z += 2.0) zCoords.push(z);
    for (let z = 140.0; z < 175.0; z += 0.25) zCoords.push(z); // Zone 1 ramp jump
    for (let z = 175.0; z < 290.0; z += 2.0) zCoords.push(z); // Flat landing
    for (let z = 290.0; z < 405.0; z += 0.25) zCoords.push(z); // Zone 2 ditches
    for (let z = 405.0; z < 415.0; z += 2.0) zCoords.push(z); // Apron
    for (let z = 415.0; z < 460.0; z += 0.25) zCoords.push(z); // Zone 3 parking hill
    for (let z = 460.0; z < 520.0; z += 0.25) zCoords.push(z); // Zone 4 side slopes
    for (let z = 520.0; z < 538.0; z += 2.0) zCoords.push(z); // Flat transition
    for (let z = 538.0; z < 582.0; z += 0.20) zCoords.push(z); // Zone 5 washboard bumps (0.20m resolution for 1.2m wave)
    for (let z = 582.0; z < 588.0; z += 2.0) zCoords.push(z); // Flat transition
    for (let z = 588.0; z < 642.0; z += 0.25) zCoords.push(z); // Zone 6 moguls field (0.25m resolution for 4m wave)
    for (let z = 642.0; z < 658.0; z += 2.0) zCoords.push(z); // Transition
    for (let z = 658.0; z < 702.0; z += 0.20) zCoords.push(z); // Zone 7 log obstacles (0.20m resolution for ~0.4m logs)
    for (let z = 702.0; z < 708.0; z += 2.0) zCoords.push(z); // Transition
    for (let z = 708.0; z < 765.0; z += 0.25) zCoords.push(z); // Zone 8 rock articulation course (0.25m resolution)
    for (let z = 765.0; z <= 780.0; z += 2.0) zCoords.push(z); // Extended flat end
    if (zCoords[zCoords.length - 1] < 780.0) zCoords.push(780.0);

    const nx = roadXCoords.length;
    const nz = zCoords.length;

    // =========================================================================
    // 2. Road Surface Mesh (Asphalt)
    // =========================================================================
    const roadPositions: number[] = [];
    const roadUvs: number[] = [];
    const roadIndices: number[] = [];

    for (let j = 0; j < nz; j++) {
      const z = zCoords[j];
      const v = (z - TestGround.ROAD_START_Z) / (TestGround.ROAD_END_Z - TestGround.ROAD_START_Z);
      for (let i = 0; i < nx; i++) {
        const x = roadXCoords[i];
        const y = this.terrainHeight(x, z);
        const u = (x + TestGround.ROAD_HALF_WIDTH) / TestGround.ROAD_WIDTH;
        roadPositions.push(x, y, z);
        roadUvs.push(u, v);
      }
    }

    for (let j = 0; j < nz - 1; j++) {
      for (let i = 0; i < nx - 1; i++) {
        const a = j * nx + i;
        const b = a + 1;
        const c = (j + 1) * nx + i;
        const d = c + 1;
        roadIndices.push(a, c, b);
        roadIndices.push(b, c, d);
      }
    }

    const roadGeom = new THREE.BufferGeometry();
    roadGeom.setAttribute('position', new THREE.Float32BufferAttribute(roadPositions, 3));
    roadGeom.setAttribute('uv', new THREE.Float32BufferAttribute(roadUvs, 2));
    roadGeom.setIndex(roadIndices);
    roadGeom.computeVertexNormals();

    const roadMat = new THREE.MeshStandardMaterial({
      color: 0x383b40, // Asphalt
      roughness: 0.85,
      metalness: 0.08,
    });
    const roadMesh = new THREE.Mesh(roadGeom, roadMat);
    roadMesh.receiveShadow = true;
    this.mesh.add(roadMesh);

    // =========================================================================
    // 3. Lane Markings (White Edge Lines & Center Dashed Line)
    // =========================================================================
    const linePositions: number[] = [];
    const lineIndices: number[] = [];
    let lineVertOffset = 0;

    const addQuadStrip = (xLeft: number, xRight: number) => {
      const startIdx = lineVertOffset;
      for (let j = 0; j < nz; j++) {
        const z = zCoords[j];
        const yL = this.terrainHeight(xLeft, z) + 0.003;
        const yR = this.terrainHeight(xRight, z) + 0.003;
        linePositions.push(xLeft, yL, z);
        linePositions.push(xRight, yR, z);
        lineVertOffset += 2;
      }
      for (let j = 0; j < nz - 1; j++) {
        const a = startIdx + j * 2;
        const b = a + 1;
        const c = startIdx + (j + 1) * 2;
        const d = c + 1;
        lineIndices.push(a, c, b);
        lineIndices.push(b, c, d);
      }
    };

    // Outer edge lines (0.25m wide)
    const edgeLineWidth = 0.25;
    addQuadStrip(-TestGround.ROAD_HALF_WIDTH, -TestGround.ROAD_HALF_WIDTH + edgeLineWidth);
    addQuadStrip(TestGround.ROAD_HALF_WIDTH - edgeLineWidth, TestGround.ROAD_HALF_WIDTH);

    // Center dashed line (0.20m wide, 3.0m dash, 3.0m gap)
    const dashLength = 3.0;
    const dashGap = 3.0;
    const dashPeriod = dashLength + dashGap;
    const dashHalfWidth = 0.10;

    for (let zStart = TestGround.ROAD_START_Z; zStart < TestGround.ROAD_END_Z; zStart += dashPeriod) {
      const zEnd = Math.min(zStart + dashLength, TestGround.ROAD_END_Z);
      const dashSteps = Math.max(2, Math.ceil((zEnd - zStart) / 0.5));
      const dashStartVert = lineVertOffset;

      for (let s = 0; s <= dashSteps; s++) {
        const z = zStart + (s / dashSteps) * (zEnd - zStart);
        const yL = this.terrainHeight(-dashHalfWidth, z) + 0.003;
        const yR = this.terrainHeight(dashHalfWidth, z) + 0.003;
        linePositions.push(-dashHalfWidth, yL, z);
        linePositions.push(dashHalfWidth, yR, z);
        lineVertOffset += 2;
      }

      for (let s = 0; s < dashSteps; s++) {
        const a = dashStartVert + s * 2;
        const b = a + 1;
        const c = dashStartVert + (s + 1) * 2;
        const d = c + 1;
        lineIndices.push(a, c, b);
        lineIndices.push(b, c, d);
      }
    }

    const lineGeom = new THREE.BufferGeometry();
    lineGeom.setAttribute('position', new THREE.Float32BufferAttribute(linePositions, 3));
    lineGeom.setIndex(lineIndices);
    lineGeom.computeVertexNormals();

    const lineMat = new THREE.MeshStandardMaterial({
      color: 0xdcd8cd,
      roughness: 0.7,
    });
    const lineMesh = new THREE.Mesh(lineGeom, lineMat);
    lineMesh.receiveShadow = true;
    this.mesh.add(lineMesh);

    // =========================================================================
    // 4. Surrounding Ground (Terrain Shoulders & Flat Plain)
    // =========================================================================
    // Left & right ground strips conform to slopes and blend smoothly to ground level (-0.015m)
    const groundLeftX = [-120.0, -50.0, -20.0, -11.0, -8.5, -7.0];
    const groundRightX = [7.0, 8.5, 11.0, 20.0, 50.0, 120.0];

    const buildGroundSide = (xArray: number[]) => {
      const gPos: number[] = [];
      const gIndices: number[] = [];
      const gxCount = xArray.length;

      for (let j = 0; j < nz; j++) {
        const z = zCoords[j];
        for (let i = 0; i < gxCount; i++) {
          const x = xArray[i];
          const y = this.terrainHeight(x, z) - 0.015;
          gPos.push(x, y, z);
        }
      }

      for (let j = 0; j < nz - 1; j++) {
        for (let i = 0; i < gxCount - 1; i++) {
          const a = j * gxCount + i;
          const b = a + 1;
          const c = (j + 1) * gxCount + i;
          const d = c + 1;
          gIndices.push(a, c, b);
          gIndices.push(b, c, d);
        }
      }

      const gGeom = new THREE.BufferGeometry();
      gGeom.setAttribute('position', new THREE.Float32BufferAttribute(gPos, 3));
      gGeom.setIndex(gIndices);
      gGeom.computeVertexNormals();

      const gMat = new THREE.MeshStandardMaterial({
        color: 0xc4beaf, // Muted sandy ground
        roughness: 0.95,
        metalness: 0.02,
      });
      const gMesh = new THREE.Mesh(gGeom, gMat);
      gMesh.receiveShadow = true;
      this.mesh.add(gMesh);
    };

    buildGroundSide(groundLeftX);
    buildGroundSide(groundRightX);

    // Front and back ground caps extending beyond road start/end
    const groundCapMat = new THREE.MeshStandardMaterial({
      color: 0xc4beaf,
      roughness: 0.95,
      metalness: 0.02,
    });

    const frontCapGeom = new THREE.PlaneGeometry(240, 60, 1, 1);
    frontCapGeom.rotateX(-Math.PI / 2);
    const frontCapMesh = new THREE.Mesh(frontCapGeom, groundCapMat);
    frontCapMesh.position.set(0, -0.015, -50.0);
    frontCapMesh.receiveShadow = true;
    this.mesh.add(frontCapMesh);

    const backCapGeom = new THREE.PlaneGeometry(240, 80, 1, 1);
    backCapGeom.rotateX(-Math.PI / 2);
    const backCapMesh = new THREE.Mesh(backCapGeom, groundCapMat);
    backCapMesh.position.set(0, -0.015, 820.0);
    backCapMesh.receiveShadow = true;
    this.mesh.add(backCapMesh);

    // =========================================================================
    // 5. Exactly One Solid 1.2m Cube at (0, 0.6, 65)
    // =========================================================================
    const obs = this.staticObstacles[0];
    const cubeGeom = new THREE.BoxGeometry(
      obs.halfExtents.x * 2,
      obs.halfExtents.y * 2,
      obs.halfExtents.z * 2
    );
    const cubeMat = new THREE.MeshStandardMaterial({
      color: 0x7a7e85, // Plain neutral concrete grey
      roughness: 0.8,
      metalness: 0.1,
    });
    const cubeMesh = new THREE.Mesh(cubeGeom, cubeMat);
    cubeMesh.position.copy(obs.center);
    cubeMesh.castShadow = true;
    cubeMesh.receiveShadow = true;
    this.mesh.add(cubeMesh);

    // =========================================================================
    // 6. Orientation Sign Boards beside the road at X = 9m
    // =========================================================================
    this.createRoadsideSign('RAMP JUMP', 'LAUNCH 1.6m @ 30-70 km/h', 9.0, 140.0);
    this.createRoadsideSign('DITCHES', 'TRENCHES & 1m DROP', 9.0, 290.0);
    this.createRoadsideSign('PARKING HILL', '20° INCLINE (36.4% GRADE)', 9.0, 415.0);
    this.createRoadsideSign('SIDE SLOPES', '15° & 25° BANKING TRACKS', 9.0, 460.0);
    this.createRoadsideSign('WASHBOARD BUMPS', 'HIGH-FREQ CHATTER (1.2m / 6cm)', 9.0, 535.0);
    this.createRoadsideSign('MOGULS ZONE', 'CROSS-AXLE ARTICULATION (4m / 15cm)', 9.0, 585.0);
    this.createRoadsideSign('LOG OBSTACLES', 'TREE TRUNK CRAWL (Ø 0.4-0.5m)', 9.0, 655.0);
    this.createRoadsideSign('ROCK ARTICULATION', 'CROSS-AXLE ARTICULATION (0.15-0.35m)', 9.0, 705.0);
  }

  /**
   * Helper to construct a roadside sign board with crisp canvas text
   */
  private createRoadsideSign(title: string, subtitle: string, x: number, z: number): void {
    const groundY = this.terrainHeight(x, z);
    const signGroup = new THREE.Group();
    signGroup.position.set(x, groundY, z);

    // Support posts (2 thin dark metal poles)
    const postMat = new THREE.MeshStandardMaterial({ color: 0x303438, roughness: 0.6 });
    const postGeom = new THREE.CylinderGeometry(0.04, 0.04, 2.2, 8);

    const post1 = new THREE.Mesh(postGeom, postMat);
    post1.position.set(-0.7, 1.1, 0);
    post1.castShadow = true;
    signGroup.add(post1);

    const post2 = new THREE.Mesh(postGeom, postMat);
    post2.position.set(0.7, 1.1, 0);
    post2.castShadow = true;
    signGroup.add(post2);

    // Signboard panel
    const boardGeom = new THREE.BoxGeometry(1.9, 0.9, 0.06);

    let faceMat: THREE.Material;
    if (typeof document !== 'undefined') {
      const canvas = document.createElement('canvas');
      canvas.width = 512;
      canvas.height = 256;
      const ctx = canvas.getContext('2d');
      if (ctx) {
        // Background
        ctx.fillStyle = '#1c2024';
        ctx.fillRect(0, 0, 512, 256);

        // Yellow border
        ctx.strokeStyle = '#f59e0b';
        ctx.lineWidth = 14;
        ctx.strokeRect(7, 7, 498, 242);

        // Warning chevron badge
        ctx.fillStyle = '#f59e0b';
        ctx.font = 'bold 26px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('TEST ZONE', 256, 55);

        // Main title
        ctx.fillStyle = '#ffffff';
        ctx.font = '900 48px sans-serif';
        ctx.fillText(title, 256, 128);

        // Subtitle
        ctx.fillStyle = '#cbd5e1';
        ctx.font = '600 24px sans-serif';
        ctx.fillText(subtitle, 256, 195);
      }

      const texture = new THREE.CanvasTexture(canvas);
      texture.minFilter = THREE.LinearFilter;
      faceMat = new THREE.MeshStandardMaterial({
        map: texture,
        roughness: 0.4,
      });
    } else {
      faceMat = new THREE.MeshStandardMaterial({
        color: 0x1c2024,
        roughness: 0.5,
      });
    }

    const backMat = new THREE.MeshStandardMaterial({
      color: 0x24282c,
      roughness: 0.8,
    });

    const boardMaterials = [
      backMat, // right
      backMat, // left
      backMat, // top
      backMat, // bottom
      faceMat, // front (+Z)
      backMat, // back (-Z)
    ];

    const boardMesh = new THREE.Mesh(boardGeom, boardMaterials);
    boardMesh.position.set(0, 1.8, 0);
    boardMesh.castShadow = true;
    // Rotate slightly inward toward road for clear visibility
    boardMesh.rotation.y = -Math.PI / 10;
    signGroup.add(boardMesh);

    this.mesh.add(signGroup);
  }
}

