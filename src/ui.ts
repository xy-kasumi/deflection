import type { BeamState } from './state';

export function bindForm(
  form: HTMLFormElement,
  state: BeamState,
  onChange: () => void,
) {
  setInput(form, 'I_mm4', String(state.I_mm4));
  setInput(form, 'J_mm4', String(state.J_mm4));
  setInput(form, 'L_mm', String(state.L_mm));
  setInput(form, 'force_kgf', String(state.force_kgf));
  setSelect(form, 'material', state.material);
  setSelect(form, 'beamType', state.beamType);

  form.addEventListener('input', () => {
    readInto(form, state);
    onChange();
  });
}

function readInto(form: HTMLFormElement, state: BeamState) {
  const data = new FormData(form);
  const I = Number(data.get('I_mm4'));
  const J = Number(data.get('J_mm4'));
  const L = Number(data.get('L_mm'));
  const F = Number(data.get('force_kgf'));
  if (Number.isFinite(I) && I > 0) state.I_mm4 = I;
  if (Number.isFinite(J) && J > 0) state.J_mm4 = J;
  if (Number.isFinite(L) && L > 0) state.L_mm = L;
  if (Number.isFinite(F)) state.force_kgf = F;
  const mat = data.get('material');
  if (mat === 'PLA' || mat === 'AL' || mat === 'STEEL') state.material = mat;
  const bt = data.get('beamType');
  if (bt === 'cantilever' || bt === 'simply-supported') state.beamType = bt;
}

function setInput(form: HTMLFormElement, name: string, value: string) {
  const el = form.elements.namedItem(name);
  if (el instanceof HTMLInputElement) el.value = value;
}

function setSelect(form: HTMLFormElement, name: string, value: string) {
  const el = form.elements.namedItem(name);
  if (el instanceof HTMLSelectElement) el.value = value;
}
