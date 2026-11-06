/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

export class InputManager {
  private keysDown: Set<string> = new Set();
  private keysJustPressed: Set<string> = new Set();

  constructor() {
    this.handleKeyDown = this.handleKeyDown.bind(this);
    this.handleKeyUp = this.handleKeyUp.bind(this);
    this.handleBlur = this.handleBlur.bind(this);

    window.addEventListener('keydown', this.handleKeyDown);
    window.addEventListener('keyup', this.handleKeyUp);
    window.addEventListener('blur', this.handleBlur);
  }

  public dispose(): void {
    window.removeEventListener('keydown', this.handleKeyDown);
    window.removeEventListener('keyup', this.handleKeyUp);
    window.removeEventListener('blur', this.handleBlur);
  }

  private handleKeyDown(e: KeyboardEvent): void {
    const key = e.key.toLowerCase();
    if (!this.keysDown.has(key)) {
      this.keysJustPressed.add(key);
    }
    this.keysDown.add(key);

    // Prevent default scrolling for Space and Arrow keys
    if ([' ', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(key)) {
      e.preventDefault();
    }
  }

  private handleKeyUp(e: KeyboardEvent): void {
    const key = e.key.toLowerCase();
    this.keysDown.delete(key);
  }

  private handleBlur(): void {
    this.keysDown.clear();
    this.keysJustPressed.clear();
  }

  public isDown(key: string): boolean {
    return this.keysDown.has(key.toLowerCase());
  }

  public wasJustPressed(key: string): boolean {
    const res = this.keysJustPressed.has(key.toLowerCase());
    if (res) {
      this.keysJustPressed.delete(key.toLowerCase());
    }
    return res;
  }

  /**
   * Clears single-frame impulse events. Called at the end of each frame.
   */
  public endFrame(): void {
    this.keysJustPressed.clear();
  }
}
