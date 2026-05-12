import { Scene } from './scene';
import { bindForm } from './ui';
import {
  computeDeflection,
  ensureVisibleScale,
  computeCrossSection,
  type DisplayScale,
} from './physics';
import { defaultState, BeamState, MATERIALS } from './state';

const state: BeamState = defaultState();
let scale: DisplayScale = 1;

const canvas = document.getElementById('canvas') as HTMLCanvasElement;
const scene = new Scene(canvas, state.L_mm);

const deltaEl = document.getElementById('readout-delta') as HTMLElement;
const matInfoEl = document.getElementById('material-info') as HTMLElement;
const form = document.getElementById('beam-form') as HTMLFormElement;

function render() {
  const defl = computeDeflection(state);
  const section = computeCrossSection(state.Ix_mm4, state.Iy_mm4, state.J_mm4);
  scene.update(state, defl, scale, section);
  deltaEl.textContent = `${fmt2sf(defl.peak_mm)} mm`;
  const m = MATERIALS[state.material];
  matInfoEl.textContent =
    `E = ${fmtModulusGPa(m.E_MPa)} GPa,  G = ${fmtModulusGPa(m.G_MPa)} GPa`;
}

function onInputChange() {
  // Only bump *up* if the current scale would render an invisible ellipse.
  // Manual clicks bypass this path so they always stick.
  const defl = computeDeflection(state);
  const bumped = ensureVisibleScale(scale, defl.peak_mm, state.L_mm);
  if (bumped !== scale) {
    scale = bumped;
    refreshScaleButtons();
  }
  render();
}

function onScaleClick(s: DisplayScale) {
  scale = s;
  refreshScaleButtons();
  render();
}

function refreshScaleButtons() {
  for (const btn of document.querySelectorAll<HTMLButtonElement>(
    '#scale-buttons button[data-scale]',
  )) {
    btn.classList.toggle('active', btn.dataset.scale === String(scale));
  }
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

bindForm(form, state, onInputChange);

for (const btn of document.querySelectorAll<HTMLButtonElement>(
  '#scale-buttons button[data-scale]',
)) {
  btn.addEventListener('click', () => {
    const v = btn.dataset.scale;
    if (v === '1' || v === '10' || v === '100' || v === '1000') {
      onScaleClick(Number(v) as DisplayScale);
    }
  });
}

refreshScaleButtons();
onInputChange();
