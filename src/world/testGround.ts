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

  // Test stations: Cube collision, Ramp Jump, Ditches, Parking Hill, Side Slopes, Washboard, Moguls, Logs, Articulation, and Surface Pack
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
    {
      id: 'surface_pack_test',
      name: 'Surface Pack & Rolling Resistance Test',
      zone: 'Zone 9: Surface Pack',
      description: '7 test strips (Z = 790..950): Asphalt, Dry Dirt, Loose Gravel, Wet Mud, Rock Face, Snow/Ice, Wet Grass, and Sand. Test friction scaling and Crr drag.',
      position: new THREE.Vector3(0, 1.06, 780),
      headingRad: 0,
    },
  ];

  // Road geometry dimensions: 14m wide, extended to Z = 970m (-20m to +970m)
  public static readonly ROAD_WIDTH = 14.0;
  public static readonly ROAD_HALF_WIDTH = 7.0;
  public static readonly ROAD_START_Z = -20.0;
  public static readonly ROAD_END_Z = 970.0;

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

  // Zone 9 Surface Pack test strips definition (20m strips from Z = 790 to Z = 950)
  public static readonly SURFACE_STRIPS: ReadonlyArray<{
    zStart: number;
    zEnd: number;
    surface: SurfaceType;
    name: string;
    description: string;
    color: number;
    roughness: number;
    metalness: number;
  }> = [
    {
      zStart: 790.0,
      zEnd: 810.0,
      surface: 'asphalt',
      name: '1. ASPHALT',
      description: 'μs 0.95 • μk 0.85 • Crr 0.015',
      color: 0x34383c, // Charcoal asphalt
      roughness: 0.85,
      metalness: 0.08,
    },
    {
      zStart: 810.0,
      zEnd: 830.0,
      surface: 'dirt',
      name: '2. DRY DIRT',
      description: 'μs 0.75 • μk 0.65 • Crr 0.035',
      color: 0x8a6c4b, // Warm dusty dirt
      roughness: 0.92,
      metalness: 0.02,
    },
    {
      zStart: 830.0,
      zEnd: 850.0,
      surface: 'gravel',
      name: '3. LOOSE GRAVEL',
      description: 'μs 0.55 • μk 0.45 • Crr 0.060',
      color: 0x767879, // Crushed granite gravel
      roughness: 0.92,
      metalness: 0.05,
    },
    {
      zStart: 850.0,
      zEnd: 870.0,
      surface: 'mud',
      name: '4. WET MUD',
      description: 'μs 0.38 • μk 0.28 • Crr 0.120',
      color: 0x362312, // Dark boggy wet mud
      roughness: 0.95,
      metalness: 0.06,
    },
    {
      zStart: 870.0,
      zEnd: 890.0,
      surface: 'rock',
      name: '5. ROCK FACE',
      description: 'μs 0.88 • μk 0.78 • Crr 0.025',
      color: 0x585d62, // Slate rock face
      roughness: 0.78,
      metalness: 0.10,
    },
    {
      zStart: 890.0,
      zEnd: 910.0,
      surface: 'ice',
      name: '6. SNOW / ICE',
      description: 'μs 0.22 • μk 0.15 • Crr 0.040',
      color: 0xdff0f8, // Slick frozen ice pale blue
      roughness: 0.12,
      metalness: 0.18,
    },
    {
      zStart: 910.0,
      zEnd: 930.0,
      surface: 'grass',
      name: '7. WET GRASS',
      description: 'μs 0.42 • μk 0.32 • Crr 0.050',
      color: 0x386a2e, // Lush wet meadow green
      roughness: 0.90,
      metalness: 0.01,
    },
    {
      zStart: 930.0,
      zEnd: 950.0,
      surface: 'sand',
      name: '8. DEEP SAND',
      description: 'μs 0.50 • μk 0.40 • Crr 0.080',
      color: 0xd6b37c, // Golden desert sand
      roughness: 0.96,
      metalness: 0.01,
    },
  ];

  constructor() {
    this.mesh = new THREE.Group();
    this.buildScene();
  }

  /**
   * Smooth analytic terrain height function using smoothstep transitions only.
   * Zero vertical faces; C1 continuity everywhere.
   * Zones 1-8 up to Z=762; flat highway strips from Z=780 to Z=970 for clean friction/Crr tests.
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
        if (z < 300.0) {
          rawH = -0.2 * smoothstep(298.8, 300.0, z);
        } else if (z <= 304.0) {
          rawH = -0.2;
        } else {
          rawH = -0.2 * (1.0 - smoothstep(304.0, 305.2, z));
        }
      } else if (z >= 318.4 && z <= 327.6) {
        if (z < 320.0) {
          rawH = -0.35 * smoothstep(318.4, 320.0, z);
        } else if (z <= 326.0) {
          rawH = -0.35;
        } else {
          rawH = -0.35 * (1.0 - smoothstep(326.0, 327.6, z));
        }
      } else if (z >= 340.0 && z <= 369.0) {
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
        rawH = 3.64;
      } else {
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
        slope = 0.268;
        if (z < 467.0) {
          zBlend = smoothstep(465.0, 467.0, z);
        } else if (z <= 483.0) {
          zBlend = 1.0;
        } else {
          zBlend = 1.0 - smoothstep(483.0, 485.0, z);
        }
      } else {
        slope = 0.466;
        if (z < 497.0) {
          zBlend = smoothstep(495.0, 497.0, z);
        } else if (z <= 513.0) {
          zBlend = 1.0;
        } else {
          zBlend = 1.0 - smoothstep(513.0, 515.0, z);
        }
      }

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
          const u = dz / log.r;
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
   * Accurately determines local SurfaceType across all road zones and Zone 9 surface strips.
   */
  public getHeightNormalSurface(x: number, z: number): GroundQuery {
    const hCenter = this.terrainHeight(x, z);
    const absX = Math.abs(x);

    // If completely outside obstacle areas or in flat Zone 9, normal is perfectly level (0, 1, 0)
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

    let surface: SurfaceType = 'dirt';

    if (onRoad) {
      if (z >= 790.0 && z <= 950.0) {
        // ZONE 9: Surface Pack Test Strips (790..950m)
        if (z < 810.0) {
          surface = 'asphalt'; // Strip 1: Asphalt
        } else if (z < 830.0) {
          surface = 'dirt';    // Strip 2: Dry Dirt
        } else if (z < 850.0) {
          surface = 'gravel';  // Strip 3: Loose Gravel
        } else if (z < 870.0) {
          surface = 'mud';     // Strip 4: Wet Mud
        } else if (z < 890.0) {
          surface = 'rock';    // Strip 5: Rock Face
        } else if (z < 910.0) {
          surface = 'ice';     // Strip 6: Snow / Ice
        } else if (z < 930.0) {
          surface = 'grass';   // Strip 7: Wet Grass
        } else {
          surface = 'sand';    // Strip 8: Deep Sand
        }
      } else if (z >= 710.0 && z <= 762.0 && hCenter > 0.05) {
        // Climbing directly on rock mound in Zone 8
        surface = 'rock';
      } else {
        surface = 'asphalt';
      }
    } else {
      surface = 'dirt';
    }

    this.queryResult.height = hCenter;
    this.queryResult.surface = surface;
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

    const dx = worldPoint.x - obs.center.x;
    const dy = worldPoint.y - obs.center.y;
    const dz = worldPoint.z - obs.center.z;

    const cosY = Math.cos(-obs.yawRad);
    const sinY = Math.sin(-obs.yawRad);
    const localX = cosY * dx - sinY * dz;
    const localZ = sinY * dx + cosY * dz;
    const localY = dy;

    const distX = obs.halfExtents.x - Math.abs(localX);
    const distY = obs.halfExtents.y - Math.abs(localY);
    const distZ = obs.halfExtents.z - Math.abs(localZ);

    if (distX > 0 && distY > 0 && distZ > 0) {
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
   * Constructs visual scene:
   * 1. Road mesh evaluated directly from terrainHeight(x, z) up to Z = 790m
   * 2. ZONE 9: 7 distinct 20m surface test strips with custom materials & colors (Z = 790..950m)
   * 3. Road runoff (Z = 950..970m)
   * 4. Transverse demarcation lines & continuous edge/center lane markings
   * 5. Surrounding terrain shoulders extended to Z = 970m
   * 6. Roadside orientation signs at X = 9m for all zones (including Zone 9 & each surface strip)
   * 7. Solid 1.2m cube at (0, 0.6, 65)
   */
  private buildScene(): void {
    // Road X coordinates: 29 points spanning -7.0m to +7.0m
    const roadXCoords: number[] = [];
    for (let i = 0; i <= 28; i++) {
      roadXCoords.push(-TestGround.ROAD_HALF_WIDTH + (i / 28) * TestGround.ROAD_WIDTH);
    }

    // Road Z coordinates for main section (Z = -20 to 790)
    const zCoords: number[] = [];
    for (let z = -20.0; z < 140.0; z += 2.0) zCoords.push(z);
    for (let z = 140.0; z < 175.0; z += 0.25) zCoords.push(z); // Zone 1 ramp jump
    for (let z = 175.0; z < 290.0; z += 2.0) zCoords.push(z); // Flat landing
    for (let z = 290.0; z < 405.0; z += 0.25) zCoords.push(z); // Zone 2 ditches
    for (let z = 405.0; z < 415.0; z += 2.0) zCoords.push(z); // Apron
    for (let z = 415.0; z < 460.0; z += 0.25) zCoords.push(z); // Zone 3 parking hill
    for (let z = 460.0; z < 520.0; z += 0.25) zCoords.push(z); // Zone 4 side slopes
    for (let z = 520.0; z < 538.0; z += 2.0) zCoords.push(z); // Flat transition
    for (let z = 538.0; z < 582.0; z += 0.20) zCoords.push(z); // Zone 5 washboard bumps
    for (let z = 582.0; z < 588.0; z += 2.0) zCoords.push(z); // Flat transition
    for (let z = 588.0; z < 642.0; z += 0.25) zCoords.push(z); // Zone 6 moguls field
    for (let z = 642.0; z < 658.0; z += 2.0) zCoords.push(z); // Transition
    for (let z = 658.0; z < 702.0; z += 0.20) zCoords.push(z); // Zone 7 log obstacles
    for (let z = 702.0; z < 708.0; z += 2.0) zCoords.push(z); // Transition
    for (let z = 708.0; z < 765.0; z += 0.25) zCoords.push(z); // Zone 8 rock articulation course
    for (let z = 765.0; z <= 790.0; z += 2.0) zCoords.push(z); // Approach to Zone 9
    if (zCoords[zCoords.length - 1] < 790.0) zCoords.push(790.0);

    const nx = roadXCoords.length;
    const nz = zCoords.length;

    // =========================================================================
    // 1. Main Road Surface Mesh (Asphalt Z = -20 to 790)
    // =========================================================================
    const roadPositions: number[] = [];
    const roadUvs: number[] = [];
    const roadIndices: number[] = [];

    for (let j = 0; j < nz; j++) {
      const z = zCoords[j];
      const v = (z - TestGround.ROAD_START_Z) / (790.0 - TestGround.ROAD_START_Z);
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
    // 2. ZONE 9: Surface Pack Test Strips (Z = 790 to 950m + 950..970 Runoff)
    // =========================================================================
    for (let s = 0; s < TestGround.SURFACE_STRIPS.length; s++) {
      const strip = TestGround.SURFACE_STRIPS[s];
      const stripGeom = new THREE.PlaneGeometry(TestGround.ROAD_WIDTH, strip.zEnd - strip.zStart, 20, 20);
      stripGeom.rotateX(-Math.PI / 2);

      const stripMat = new THREE.MeshStandardMaterial({
        color: strip.color,
        roughness: strip.roughness,
        metalness: strip.metalness,
      });

      const stripMesh = new THREE.Mesh(stripGeom, stripMat);
      stripMesh.position.set(0, 0.0, (strip.zStart + strip.zEnd) * 0.5);
      stripMesh.receiveShadow = true;
      this.mesh.add(stripMesh);

      // Roadside marker sign for each strip
      this.createRoadsideSign(strip.name, strip.description, 9.0, (strip.zStart + strip.zEnd) * 0.5);

      // Transverse boundary warning stripe line at start of strip
      const boundaryGeom = new THREE.PlaneGeometry(TestGround.ROAD_WIDTH, 0.4);
      boundaryGeom.rotateX(-Math.PI / 2);
      const boundaryMat = new THREE.MeshStandardMaterial({
        color: 0xf59e0b, // Amber border
        roughness: 0.5,
      });
      const boundaryMesh = new THREE.Mesh(boundaryGeom, boundaryMat);
      boundaryMesh.position.set(0, 0.004, strip.zStart);
      this.mesh.add(boundaryMesh);
    }

    // Runoff Strip (Z = 950..970m)
    const runoffGeom = new THREE.PlaneGeometry(TestGround.ROAD_WIDTH, 20.0, 10, 10);
    runoffGeom.rotateX(-Math.PI / 2);
    const runoffMesh = new THREE.Mesh(runoffGeom, roadMat);
    runoffMesh.position.set(0, 0.0, 960.0);
    runoffMesh.receiveShadow = true;
    this.mesh.add(runoffMesh);

    // Final boundary line at Z = 950
    const finalBoundaryGeom = new THREE.PlaneGeometry(TestGround.ROAD_WIDTH, 0.4);
    finalBoundaryGeom.rotateX(-Math.PI / 2);
    const finalBoundaryMat = new THREE.MeshStandardMaterial({ color: 0xf59e0b, roughness: 0.5 });
    const finalBoundaryMesh = new THREE.Mesh(finalBoundaryGeom, finalBoundaryMat);
    finalBoundaryMesh.position.set(0, 0.004, 950.0);
    this.mesh.add(finalBoundaryMesh);

    // =========================================================================
    // 3. Lane Markings (White Edge Lines & Center Dashed Line to Z = 970m)
    // =========================================================================
    const linePositions: number[] = [];
    const lineIndices: number[] = [];
    let lineVertOffset = 0;

    // Full dense Z list extending to Z = 970m
    const allZCoords = [...zCoords];
    for (let z = 792.0; z <= TestGround.ROAD_END_Z; z += 2.0) {
      allZCoords.push(z);
    }
    const totalNz = allZCoords.length;

    const addQuadStrip = (xLeft: number, xRight: number) => {
      const startIdx = lineVertOffset;
      for (let j = 0; j < totalNz; j++) {
        const z = allZCoords[j];
        const yL = this.terrainHeight(xLeft, z) + 0.003;
        const yR = this.terrainHeight(xRight, z) + 0.003;
        linePositions.push(xLeft, yL, z);
        linePositions.push(xRight, yR, z);
        lineVertOffset += 2;
      }
      for (let j = 0; j < totalNz - 1; j++) {
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
    // 4. Surrounding Ground (Terrain Shoulders & Flat Plain to Z = 970m)
    // =========================================================================
    const groundLeftX = [-120.0, -50.0, -20.0, -11.0, -8.5, -7.0];
    const groundRightX = [7.0, 8.5, 11.0, 20.0, 50.0, 120.0];

    const buildGroundSide = (xArray: number[]) => {
      const gPos: number[] = [];
      const gIndices: number[] = [];
      const gxCount = xArray.length;

      for (let j = 0; j < totalNz; j++) {
        const z = allZCoords[j];
        for (let i = 0; i < gxCount; i++) {
          const x = xArray[i];
          const y = this.terrainHeight(x, z) - 0.015;
          gPos.push(x, y, z);
        }
      }

      for (let j = 0; j < totalNz - 1; j++) {
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

    // Front and back ground caps
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
    backCapMesh.position.set(0, -0.015, 1010.0);
    backCapMesh.receiveShadow = true;
    this.mesh.add(backCapMesh);

    // =========================================================================
    // 5. Solid 1.2m Cube at (0, 0.6, 65)
    // =========================================================================
    const obs = this.staticObstacles[0];
    const cubeGeom = new THREE.BoxGeometry(
      obs.halfExtents.x * 2,
      obs.halfExtents.y * 2,
      obs.halfExtents.z * 2
    );
    const cubeMat = new THREE.MeshStandardMaterial({
      color: 0x7a7e85, // Neutral concrete grey
      roughness: 0.8,
      metalness: 0.1,
    });
    const cubeMesh = new THREE.Mesh(cubeGeom, cubeMat);
    cubeMesh.position.copy(obs.center);
    cubeMesh.castShadow = true;
    cubeMesh.receiveShadow = true;
    this.mesh.add(cubeMesh);

    // =========================================================================
    // 6. Zone Entry Orientation Signs beside road at X = 9m
    // =========================================================================
    this.createRoadsideSign('RAMP JUMP', 'LAUNCH 1.6m @ 30-70 km/h', 9.0, 140.0);
    this.createRoadsideSign('DITCHES', 'TRENCHES & 1m DROP', 9.0, 290.0);
    this.createRoadsideSign('PARKING HILL', '20° INCLINE (36.4% GRADE)', 9.0, 415.0);
    this.createRoadsideSign('SIDE SLOPES', '15° & 25° BANKING TRACKS', 9.0, 460.0);
    this.createRoadsideSign('WASHBOARD BUMPS', 'HIGH-FREQ CHATTER (1.2m / 6cm)', 9.0, 535.0);
    this.createRoadsideSign('MOGULS ZONE', 'CROSS-AXLE ARTICULATION (4m / 15cm)', 9.0, 585.0);
    this.createRoadsideSign('LOG OBSTACLES', 'TREE TRUNK CRAWL (Ø 0.4-0.5m)', 9.0, 655.0);
    this.createRoadsideSign('ROCK ARTICULATION', 'CROSS-AXLE ARTICULATION (0.15-0.35m)', 9.0, 705.0);
    this.createRoadsideSign('SURFACE PACK', 'ZONE 9: 7 SURFACE STRIPS', 9.0, 785.0);
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
        ctx.fillStyle = '#1c2024';
        ctx.fillRect(0, 0, 512, 256);

        ctx.strokeStyle = '#f59e0b';
        ctx.lineWidth = 14;
        ctx.strokeRect(7, 7, 498, 242);

        ctx.fillStyle = '#f59e0b';
        ctx.font = 'bold 26px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('TEST ZONE', 256, 55);

        ctx.fillStyle = '#ffffff';
        ctx.font = '900 46px sans-serif';
        ctx.fillText(title, 256, 128);

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
      backMat,
      backMat,
      backMat,
      backMat,
      faceMat,
      backMat,
    ];

    const boardMesh = new THREE.Mesh(boardGeom, boardMaterials);
    boardMesh.position.set(0, 1.8, 0);
    boardMesh.castShadow = true;
    boardMesh.rotation.y = -Math.PI / 10;
    signGroup.add(boardMesh);

    this.mesh.add(signGroup);
  }
}
