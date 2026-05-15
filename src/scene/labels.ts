import * as THREE from 'three';

// DOM overlay for floating labels anchored to world-space points. Each label
// is a `.scene-label` div positioned per frame by projecting its world point
// through the active camera. The overlay sits in the canvas's parent so the
// labels can extend past the WebGL canvas without clipping.

interface LabelEntry {
  el: HTMLDivElement;
  worldPos: THREE.Vector3;
}

export class Labels {
  private layer: HTMLDivElement;
  private entries: LabelEntry[] = [];

  constructor(container: HTMLElement) {
    this.layer = document.createElement('div');
    this.layer.className = 'scene-labels';
    container.appendChild(this.layer);
  }

  add(
    text: string,
    worldPos: THREE.Vector3,
    opts: { classes?: string[]; onClick?: () => void } = {},
  ): void {
    const el = document.createElement('div');
    el.className = 'scene-label';
    if (opts.classes) for (const c of opts.classes) el.classList.add(c);
    el.textContent = text;
    if (opts.onClick) el.addEventListener('click', opts.onClick);
    this.layer.appendChild(el);
    this.entries.push({ el, worldPos: worldPos.clone() });
  }

  clear(): void {
    for (const e of this.entries) e.el.remove();
    this.entries = [];
    // Don't carry a transient hover-driven fade across a rebuild.
    this.layer.classList.remove('faded');
  }

  // Temporary hide for all labels (e.g. while a contribution segment is on
  // screen and the δ-text would occlude it). Toggles a class on the layer
  // so the per-label classes (selected/hidden) are untouched.
  setFaded(faded: boolean): void {
    this.layer.classList.toggle('faded', faded);
  }

  update(camera: THREE.Camera, canvas: HTMLCanvasElement): void {
    if (this.entries.length === 0) return;
    const rect = canvas.getBoundingClientRect();
    const w = rect.width;
    const h = rect.height;
    const v = new THREE.Vector3();
    for (const e of this.entries) {
      v.copy(e.worldPos).project(camera);
      const x = (v.x * 0.5 + 0.5) * w;
      const y = (-v.y * 0.5 + 0.5) * h;
      e.el.style.transform = `translate(-50%, -50%) translate(${x}px, ${y}px)`;
    }
  }
}
