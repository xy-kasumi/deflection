import { Scene, ViewPreset } from './scene';
import { bindForm } from './ui';
import {
  computeDeflection,
  autoDisplayScale,
  computeCrossSection,
  type DisplayScale,
} from './physics';
import { defaultState, BeamState, MATERIALS } from './state';

type ScaleMode = 'auto' | DisplayScale;

const state: BeamState = defaultState();
let scaleMode: ScaleMode = 'auto';

const canvas = document.getElementById('canvas') as HTMLCanvasElement;
const scene = new Scene(canvas, state.L_mm);

const deltaEl = document.getElementById('readout-delta') as HTMLElement;
const scaleEl = document.getElementById('readout-scale') as HTMLElement;
const matInfoEl = document.getElementById('material-info') as HTMLElement;
const form = document.getElementById('beam-form') as HTMLFormElement;

function recompute() {
  const defl = computeDeflection(state);
  const scale: DisplayScale =
    scaleMode === 'auto'
      ? autoDisplayScale(defl.peak_mm, state.L_mm)
      : scaleMode;
  const section = computeCrossSection(state.Ix_mm4, state.Iy_mm4, state.J_mm4);

  scene.update(state, defl, scale, section);
  deltaEl.textContent = `${fmt2sf(defl.peak_mm)} mm`;
  scaleEl.textContent =
    scaleMode === 'auto' ? `auto (×${scale})` : `×${scale}`;

  const m = MATERIALS[state.material];
  matInfoEl.textContent =
    `E = ${fmtModulusGPa(m.E_MPa)} GPa,  G = ${fmtModulusGPa(m.G_MPa)} GPa`;
}

function fmt2sf(x: number): string {
  if (!Number.isFinite(x) || x === 0) return '0';
  const exp = Math.floor(Math.log10(Math.abs(x)));
  const step = Math.pow(10, exp - 1);
  const rounded = Math.round(x / step) * step;
  const abs = Math.abs(rounded);
  if (abs >= 10)    return rounded.toFixed(0);
  if (abs >= 1)     return rounded.toFixed(1);
  if (abs >= 0.1)   return rounded.toFixed(2);
  if (abs >= 0.01)  return rounded.toFixed(3);
  if (abs >= 0.001) return rounded.toFixed(4);
  return rounded.toExponential(1);
}

function fmtModulusGPa(mpa: number): string {
  const gpa = mpa / 1000;
  return gpa >= 10 ? gpa.toFixed(0) : gpa.toFixed(1);
}

function setScaleMode(mode: ScaleMode) {
  scaleMode = mode;
  for (const btn of document.querySelectorAll<HTMLButtonElement>(
    '#scale-buttons button[data-scale]',
  )) {
    btn.classList.toggle('active', btn.dataset.scale === String(mode));
  }
  recompute();
}

bindForm(form, state, recompute);

for (const btn of document.querySelectorAll<HTMLButtonElement>(
  '#view-buttons button[data-view]',
)) {
  btn.addEventListener('click', () => {
    scene.setView(btn.dataset.view as ViewPreset);
  });
}
for (const btn of document.querySelectorAll<HTMLButtonElement>(
  '#view-buttons button[data-yaw]',
)) {
  const delta = Number(btn.dataset.yaw);
  btn.addEventListener('click', () => scene.nudgeYawDeg(delta));
}
for (const btn of document.querySelectorAll<HTMLButtonElement>(
  '#scale-buttons button[data-scale]',
)) {
  btn.addEventListener('click', () => {
    const v = btn.dataset.scale;
    if (v === 'auto') setScaleMode('auto');
    else if (v === '1' || v === '10' || v === '100') {
      setScaleMode(Number(v) as DisplayScale);
    }
  });
}

setScaleMode('auto');
