import type { BeamDef, Param } from '../dsl/parse';

// Resolved section properties. A = null means area was not derivable
// (section(...) without an explicit A) — mass_accel body load is then
// skipped for this beam.
export interface Section {
  A_mm2: number | null;
  Ix_mm4: number;
  Iy_mm4: number;
  J_mm4: number;
}

// Default when shape is missing entirely: a small solid square.
const DEFAULT_SECTION: Section = (() => {
  const W = 10, H = 10;
  return {
    A_mm2: W * H,
    Ix_mm4: (W * H ** 3) / 12,
    Iy_mm4: (W ** 3 * H) / 12,
    J_mm4: rectJ(W, H),
  };
})();

export function resolveSection(beam: BeamDef): Section {
  const shape = findShape(beam.params);
  if (!shape) return DEFAULT_SECTION;

  if (shape.name === 'rect')    return rectSection(shape.params ?? []);
  if (shape.name === 'round')   return roundSection(shape.params ?? []);
  if (shape.name === 'section') return explicitSection(shape.params ?? []);
  return DEFAULT_SECTION;
}

function findShape(params: Param[]): Extract<Param, { kind: 'ident' }> | null {
  for (const p of params) {
    if (
      p.kind === 'ident' &&
      p.params !== undefined &&
      (p.name === 'rect' || p.name === 'round' || p.name === 'section')
    ) {
      return p;
    }
  }
  return null;
}

function findQ(params: Param[], prefix: string): number | null {
  for (const p of params) {
    if (p.kind === 'quantity' && p.quantity.prefix === prefix) {
      return p.quantity.value;
    }
  }
  return null;
}

function rectSection(args: Param[]): Section {
  const W = findQ(args, 'W');
  const H = findQ(args, 'H');
  if (W === null || H === null) return DEFAULT_SECTION;
  const T = findQ(args, 'T');
  if (T !== null && 2 * T < W && 2 * T < H) {
    // Hollow rectangular tube.
    const Wi = W - 2 * T;
    const Hi = H - 2 * T;
    return {
      A_mm2: W * H - Wi * Hi,
      Ix_mm4: (W * H ** 3 - Wi * Hi ** 3) / 12,
      Iy_mm4: (W ** 3 * H - Wi ** 3 * Hi) / 12,
      J_mm4: thinWalledBoxJ(W, H, T),
    };
  }
  return {
    A_mm2: W * H,
    Ix_mm4: (W * H ** 3) / 12,
    Iy_mm4: (W ** 3 * H) / 12,
    J_mm4: rectJ(W, H),
  };
}

function roundSection(args: Param[]): Section {
  const D = findQ(args, 'D');
  if (D === null) return DEFAULT_SECTION;
  const T = findQ(args, 'T');
  if (T !== null && 2 * T < D) {
    const Di = D - 2 * T;
    return {
      A_mm2: (Math.PI / 4) * (D * D - Di * Di),
      Ix_mm4: (Math.PI / 64) * (D ** 4 - Di ** 4),
      Iy_mm4: (Math.PI / 64) * (D ** 4 - Di ** 4),
      J_mm4: (Math.PI / 32) * (D ** 4 - Di ** 4),
    };
  }
  return {
    A_mm2: (Math.PI / 4) * D * D,
    Ix_mm4: (Math.PI / 64) * D ** 4,
    Iy_mm4: (Math.PI / 64) * D ** 4,
    J_mm4: (Math.PI / 32) * D ** 4,
  };
}

function explicitSection(args: Param[]): Section {
  const Ix = findQ(args, 'Ix');
  const Iy = findQ(args, 'Iy');
  const I  = findQ(args, 'I');
  const J  = findQ(args, 'J');
  const A  = findQ(args, 'A');
  if (J === null) return DEFAULT_SECTION;
  const IxOut = Ix !== null ? Ix : (I !== null ? I : DEFAULT_SECTION.Ix_mm4);
  const IyOut = Iy !== null ? Iy : (I !== null ? I : DEFAULT_SECTION.Iy_mm4);
  const AOut = A !== null && A > 0 ? A : null;
  return { A_mm2: AOut, Ix_mm4: IxOut, Iy_mm4: IyOut, J_mm4: J };
}

// Solid rectangle torsion constant (Roark, well-known approximation).
// a = longer side, b = shorter side; returns J [mm^4].
function rectJ(W: number, H: number): number {
  const a = Math.max(W, H);
  const b = Math.min(W, H);
  const r = b / a;
  return a * b ** 3 * (1 / 3 - 0.21 * r * (1 - r ** 4 / 12));
}

// Thin-walled closed rectangular tube torsion (Bredt's formula): J = 4·Ω²/∮(ds/t),
// for constant wall thickness t and centerline rectangle (W-t) × (H-t).
function thinWalledBoxJ(W: number, H: number, t: number): number {
  const Wm = W - t;
  const Hm = H - t;
  return (2 * t * Wm * Wm * Hm * Hm) / (Wm + Hm);
}
