import type { BeamState } from './state';

export function bindForm(
  form: HTMLFormElement,
  state: BeamState,
  onChange: () => void,
) {
  setInput(form, 'Ix_mm4', String(state.Ix_mm4));
  setInput(form, 'Iy_mm4', String(state.Iy_mm4));
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
  const Ix = Number(data.get('Ix_mm4'));
  const Iy = Number(data.get('Iy_mm4'));
  const J = Number(data.get('J_mm4'));
  const L = Number(data.get('L_mm'));
  const F = Number(data.get('force_kgf'));
  if (Number.isFinite(Ix) && Ix > 0) state.Ix_mm4 = Ix;
  if (Number.isFinite(Iy) && Iy > 0) state.Iy_mm4 = Iy;
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
