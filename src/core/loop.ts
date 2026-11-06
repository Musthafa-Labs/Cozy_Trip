/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

export class FixedTimestepLoop {
  public readonly targetHz: number = 120;
  public readonly fixedDt: number = 1 / 120; // 0.008333s per sub-step
  public readonly maxSubSteps: number = 8;    // Prevent spiral of death on long frames

  private accumulator: number = 0;
  private lastTime: number = 0;
  private isRunning: boolean = false;
  private animFrameId: number | null = null;

  private onFixedUpdate: (dt: number) => void;
  private onRender: (alpha: number, dt: number) => void;

  constructor(
    onFixedUpdate: (dt: number) => void,
    onRender: (alpha: number, dt: number) => void
  ) {
    this.onFixedUpdate = onFixedUpdate;
    this.onRender = onRender;
  }

  public start(): void {
    if (this.isRunning) return;
    this.isRunning = true;
    this.lastTime = performance.now() * 0.001;
    this.accumulator = 0;
    this.loop = this.loop.bind(this);
    this.animFrameId = requestAnimationFrame(this.loop);
  }

  public stop(): void {
    this.isRunning = false;
    if (this.animFrameId !== null) {
      cancelAnimationFrame(this.animFrameId);
      this.animFrameId = null;
    }
  }

  private loop(currentTimeMs: number): void {
    if (!this.isRunning) return;

    const currentTime = currentTimeMs * 0.001;
    let frameDelta = currentTime - this.lastTime;
    this.lastTime = currentTime;

    // Clamp huge frame hitch (e.g. tab switch) to 0.1s
    if (frameDelta > 0.1) {
      frameDelta = 0.1;
    }

    this.accumulator += frameDelta;
    let subSteps = 0;

    while (this.accumulator >= this.fixedDt && subSteps < this.maxSubSteps) {
      this.onFixedUpdate(this.fixedDt);
      this.accumulator -= this.fixedDt;
      subSteps++;
    }

    // Alpha for rendering interpolation between last state and current state
    const alpha = Math.min(1.0, this.accumulator / this.fixedDt);
    this.onRender(alpha, frameDelta);

    this.animFrameId = requestAnimationFrame(this.loop);
  }
}
