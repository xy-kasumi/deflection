import type { Directional } from './sim/directional';
import type { Vec3 } from './sim/problem';

// Equirectangular δ(dir) picker. A heatmap over the full sphere — x = azimuth
// θ about world-up (+Y), y = polar angle φ from +Y, so the canvas's vertical
// axis is world-up — that the user clicks to pick a direction.
//
// The heatmap is cached as ImageData and recomputed only when the δ field
// changes, so re-marking (e.g. during a drag) is just a putImageData + a
// circle, no per-pixel δ evaluation.

const W = 256;
const H = 128;

export class DirPicker {
  readonly el: HTMLCanvasElement;
  private ctx: CanvasRenderingContext2D;
  private deflection: Directional | null = null;
  private heatmap: ImageData | null = null;
  private pick: Vec3 | null = null;
  /** Called with a unit direction when the user picks one. */
  onPick: (d: Vec3) => void = () => {};

  constructor() {
    this.el = document.createElement('canvas');
    this.el.className = 'bd-picker';
    this.el.width = W;
    this.el.height = H;
    this.ctx = this.el.getContext('2d')!;
    this.installInput();
  }

  // Point the picker at a δ field. Recomputes the heatmap only when it
  // changes, so this is cheap to call on every redraw.
  setData(deflection: Directional): void {
    if (deflection === this.deflection) return;
    this.deflection = deflection;
    this.heatmap = this.computeHeatmap(deflection);
    this.render();
  }

  // Mark the current pick (null → the field's auto argmax d*).
  setPick(dir: Vec3 | null): void {
    this.pick = dir;
    this.render();
  }

  private render(): void {
    if (!this.heatmap || !this.deflection) return;
    this.ctx.putImageData(this.heatmap, 0, 0);
    const [mx, my] = project(this.pick ?? this.deflection.max().dir);
    for (const [color, width] of [['#fff', 2], ['#000', 1]] as const) {
      this.ctx.lineWidth = width;
      this.ctx.strokeStyle = color;
      this.ctx.beginPath();
      this.ctx.arc(mx, my, 4, 0, Math.PI * 2);
      this.ctx.stroke();
    }
  }

  private computeHeatmap(deflection: Directional): ImageData {
    const dmax = deflection.max().value || 1;
    const img = this.ctx.createImageData(W, H);
    for (let py = 0; py < H; py++) {
      for (let px = 0; px < W; px++) {
        const t = deflection.at(unproject(px + 0.5, py + 0.5)) / dmax;
        const [r, g, b] = ramp(t);
        const o = (py * W + px) * 4;
        img.data[o] = r;
        img.data[o + 1] = g;
        img.data[o + 2] = b;
        img.data[o + 3] = 255;
      }
    }
    return img;
  }

  private installInput(): void {
    this.el.addEventListener('click', (e) => this.onPick(this.dirAt(e)));
  }

  private dirAt(e: { clientX: number; clientY: number }): Vec3 {
    const rect = this.el.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * W;
    const py = ((e.clientY - rect.top) / rect.height) * H;
    return unproject(px, py);
  }
}

// Canvas pixel → world direction. θ ∈ [0,2π) about +Y, φ ∈ [0,π] from +Y.
function unproject(px: number, py: number): Vec3 {
  const theta = (px / W) * 2 * Math.PI;
  const phi = (py / H) * Math.PI;
  const s = Math.sin(phi);
  return [s * Math.cos(theta), Math.cos(phi), s * Math.sin(theta)];
}

// World direction → canvas pixel (inverse of unproject).
function project(d: Vec3): [number, number] {
  const phi = Math.acos(Math.max(-1, Math.min(1, d[1])));
  let theta = Math.atan2(d[2], d[0]);
  if (theta < 0) theta += 2 * Math.PI;
  return [(theta / (2 * Math.PI)) * W, (phi / Math.PI) * H];
}

// δ heatmap ramp: panel-light at 0 → warm amber at the peak. Deflection's
// domain colour is warm (the scene lobes too); red stays reserved for errors.
function ramp(t: number): [number, number, number] {
  const c = Math.max(0, Math.min(1, t));
  return [
    Math.round(246 + (232 - 246) * c),
    Math.round(247 + (148 - 247) * c),
    Math.round(249 + (60 - 249) * c),
  ];
}
