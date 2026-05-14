// The seam between the DSL/UI side and the sim/ library. Builds sim/'s
// self-contained `Problem` from the DSL `Structure` + walker's `BeamNode[]`,
// and retains the load-index → DSL-span / mass association the UI needs.
//
// This is the only module that depends on both `dsl/` and `sim/`.

import type { Diagnostic, Span } from './dsl/diagnostics';
import type { Attachment, BeamDef, Param, Structure } from './dsl/parse';
import type { BeamNode } from './walker';
import type {
  Beam,
  DeflectionQuery,
  Frame,
  Load,
  Material,
  Problem,
  Section,
  Vec3,
} from './sim/problem';
import type { SimError } from './sim/simulate';

// Per-load provenance, parallel to `Problem.loads`. The sim result indexes
// loads by integer; this lets the UI recover where each load came from.
export interface LoadProvenance {
  source: 'user' | 'mass_accel';
  sourceSpan: Span | null; // user loads: the load() attachment span
  mass_kg: number | null; // mass_accel loads: the body mass
}

export interface LoadSystem {
  problem: Problem;
  loadProvenance: LoadProvenance[];
  tipQueryIx: number; // index into problem.queries, or -1
  supportKind: 'single' | 'both' | undefined; // raw, for scene.update
  supportSpan: Span | null; // span of the support() env, for error mapping
  diagnostics: Diagnostic[]; // conversion-time issues (e.g. section without A)
}

export function buildLoadSystem(structure: Structure, beams: BeamNode[]): LoadSystem {
  const diagnostics: Diagnostic[] = [];

  const supportKind = getSupportKind(structure);
  const support: 'single' | 'both' = supportKind ?? 'single';
  const supportSpan = structure.envs.find((e) => e.name === 'support')?.span ?? null;

  // Per-beam resolved section (incl. area, kept here for body loads) + material.
  const sections = beams.map((b) => resolveSection(b.def));
  const matIds: MaterialId[] = beams.map((b) =>
    isKnownMaterial(b.material) ? (b.material as MaterialId) : 'plastic',
  );
  const mats = matIds.map((id) => MATERIALS[id]);

  const simBeams: Beam[] = beams.map((b, i) => ({
    frame: toSimFrame(b),
    length_mm: b.length_mm,
    section: { Ix_mm4: sections[i]!.Ix_mm4, Iy_mm4: sections[i]!.Iy_mm4, J_mm4: sections[i]!.J_mm4 },
    material: { E_MPa: mats[i]!.E_MPa, G_MPa: mats[i]!.G_MPa } satisfies Material,
  }));

  // Loads: explicit load() attachments first (chain order), then mass_accel
  // body loads — matches the order the old sim built them in.
  const loads: Load[] = [];
  const loadProvenance: LoadProvenance[] = [];

  for (let i = 0; i < beams.length; i++) {
    for (const att of beams[i]!.attachmentOffsets) {
      if (att.def.name !== 'load') continue;
      const F = loadMagnitudeN(att.def);
      if (F <= 0) continue;
      loads.push({ beamIx: i, offset_mm: att.local_mm, Fmax_N: F });
      loadProvenance.push({ source: 'user', sourceSpan: att.def.span, mass_kg: null });
    }
  }

  // mass_accel(a): one direction-free mid-beam body load per beam. The seam
  // owns the rho·A·L·a arithmetic so sim/ never sees density or area.
  const accel_m_s2 = getMassAccel_m_s2(structure);
  if (accel_m_s2 > 0) {
    for (let i = 0; i < beams.length; i++) {
      const b = beams[i]!;
      const A = sections[i]!.A_mm2;
      if (A === null || A <= 0) {
        diagnostics.push({
          severity: 'warning',
          message:
            'section(...) without A: no mass_accel body load for this beam (add A=… to include self-weight)',
          span: b.def.span,
        });
        continue;
      }
      const mass_kg = mats[i]!.rho_kg_per_mm3 * A * b.length_mm;
      const F_N = mass_kg * accel_m_s2;
      if (F_N <= 0) continue;
      loads.push({ beamIx: i, offset_mm: b.length_mm / 2, Fmax_N: F_N });
      loadProvenance.push({ source: 'mass_accel', sourceSpan: null, mass_kg });
    }
  }

  const { queries, tipQueryIx } = buildQueries(beams, support);

  return {
    problem: { beams: simBeams, loads, queries, support },
    loadProvenance,
    tipQueryIx,
    supportKind,
    supportSpan,
    diagnostics,
  };
}

// Maps a sim/ failure back to an editor diagnostic. support-singular points at
// the support() env; the validation errors are caller-side bugs (walker always
// builds connected, origin-clamped chains) so they get a doc-start span.
export function simErrorToDiagnostic(err: SimError, ls: LoadSystem): Diagnostic {
  const span =
    err.code === 'support-singular'
      ? ls.supportSpan ?? { start: 0, end: 0 }
      : { start: 0, end: 0 };
  return { severity: 'error', message: err.message, span };
}

// ---------- frame mapping ----------

// walker frame {fwd, right, up} → sim frame {axial, ex, ey}.
function toSimFrame(b: BeamNode): Frame {
  const f = b.startFrame;
  return { origin_mm: f.origin, ex: f.right, ey: f.up, axial: f.fwd };
}

// ---------- queries (was sim/compliance.ts buildQueryNodes + getTipLoc) ----------

const QUERY_POS_EPS = 1e-6;

function buildQueries(
  beams: BeamNode[],
  support: 'single' | 'both',
): { queries: DeflectionQuery[]; tipQueryIx: number } {
  // Candidate query points: each beam's start and end, deduped by world
  // position (a beam-end coinciding with the next beam's start counts once);
  // the clamped origin is dropped.
  let queries: DeflectionQuery[] = [];
  const seen: Vec3[] = [];
  for (let i = 0; i < beams.length; i++) {
    const b = beams[i]!;
    const endpoints: { offset_mm: number; world: Vec3 }[] = [
      { offset_mm: 0, world: b.startFrame.origin },
      {
        offset_mm: b.length_mm,
        world: addV(b.startFrame.origin, scaleV(b.startFrame.fwd, b.length_mm)),
      },
    ];
    for (const e of endpoints) {
      if (Math.hypot(e.world[0], e.world[1], e.world[2]) < QUERY_POS_EPS) continue; // origin
      if (seen.some((sp) => dist(sp, e.world) < QUERY_POS_EPS)) continue;
      queries.push({ beamIx: i, offset_mm: e.offset_mm });
      seen.push(e.world);
    }
  }

  // Tip per the vocab: end of last beam, except mid of root beam under
  // support(both) with a single beam (both endpoints clamped). Add explicitly
  // so the UI headline always has an anchor.
  const tipLoc = getTipLoc(beams, support);
  if (
    tipLoc &&
    !queries.some((q) => q.beamIx === tipLoc.beamIx && q.offset_mm === tipLoc.offset_mm)
  ) {
    queries.push(tipLoc);
  }

  // Under support(both) the root beam's end is the second clamp point —
  // deflection there is zero by construction, so it's not a reported query.
  // sim/ re-adds it internally for the fixed-fixed solve.
  if (support === 'both' && beams.length > 0) {
    const rootLen = beams[0]!.length_mm;
    queries = queries.filter((q) => !(q.beamIx === 0 && q.offset_mm === rootLen));
  }

  const tipQueryIx = tipLoc
    ? queries.findIndex((q) => q.beamIx === tipLoc.beamIx && q.offset_mm === tipLoc.offset_mm)
    : -1;
  return { queries, tipQueryIx };
}

function getTipLoc(
  beams: BeamNode[],
  support: 'single' | 'both',
): DeflectionQuery | null {
  if (beams.length === 0) return null;
  if (support === 'both' && beams.length === 1) {
    return { beamIx: 0, offset_mm: beams[0]!.length_mm / 2 };
  }
  const last = beams.length - 1;
  return { beamIx: last, offset_mm: beams[last]!.length_mm };
}

// ---------- env interpretation (was sim/compliance.ts) ----------

function getSupportKind(structure: Structure): 'single' | 'both' | undefined {
  for (const env of structure.envs) {
    if (env.name !== 'support') continue;
    const arg = env.params[0];
    if (arg && arg.kind === 'ident' && (arg.name === 'single' || arg.name === 'both')) {
      return arg.name;
    }
  }
  return undefined;
}

// Acceleration in m/s². Defaults to 1G when no `mass_accel` env is present.
// Bare numbers and `G`-unit values are scaled by g; `m/s2`-unit values pass
// through unchanged. Unknown units fall back to scaling by g.
function getMassAccel_m_s2(structure: Structure): number {
  let env;
  for (const e of structure.envs) {
    if (e.name === 'mass_accel') {
      env = e;
      break;
    }
  }
  if (!env || env.params.length !== 1) return G_M_PER_S2;
  const p = env.params[0]!;
  if (p.kind !== 'quantity') return G_M_PER_S2;
  const { value, unit } = p.quantity;
  if (!Number.isFinite(value) || value < 0) return G_M_PER_S2;
  if (unit === 'm/s2') return value;
  return value * G_M_PER_S2;
}

// ---------- materials & unit constants ----------

export type MaterialId = 'plastic' | 'aluminum' | 'steel';

interface MaterialProps {
  E_MPa: number;            // Young's modulus  [N/mm^2]
  G_MPa: number;            // Shear modulus    [N/mm^2]
  rho_kg_per_mm3: number;   // Density          [kg/mm^3]
}

// Categorical single-value approximations — these are typical numbers, not
// specific alloys. Suitable for back-of-envelope (±20%) stiffness sizing.
const MATERIALS: Record<MaterialId, MaterialProps> = {
  plastic:  { E_MPa:   3_500, G_MPa:  1_300, rho_kg_per_mm3: 1.20e-6 },
  aluminum: { E_MPa:  70_000, G_MPa: 26_000, rho_kg_per_mm3: 2.70e-6 },
  steel:    { E_MPa: 200_000, G_MPa: 79_000, rho_kg_per_mm3: 7.85e-6 },
};

const KGF_TO_N = 9.80665;
const G_M_PER_S2 = 9.80665;

// ---------- load magnitude (was sim/compliance.ts) ----------

function loadMagnitudeN(att: Attachment): number {
  if (att.params.length !== 1) return 0;
  const p = att.params[0]!;
  if (p.kind !== 'quantity') return 0;
  const v = p.quantity.value;
  if (!Number.isFinite(v) || v <= 0) return 0;
  const unit = p.quantity.unit ?? 'kgf';
  if (unit === 'N') return v;
  if (unit === 'kgf') return v * KGF_TO_N;
  return v * KGF_TO_N;
}

function isKnownMaterial(s: string | undefined): boolean {
  return s === 'plastic' || s === 'aluminum' || s === 'steel';
}

// ---------- section resolution (was sim/section.ts) ----------
//
// Parses a beam's shape params into cross-section properties. Area is kept here
// (for mass_accel body loads); only Ix/Iy/J flow into sim/'s `Section`.

interface ResolvedSection extends Section {
  A_mm2: number | null;
}

// Default when shape is missing entirely: a small solid square.
const DEFAULT_SECTION: ResolvedSection = (() => {
  const W = 10,
    H = 10;
  return {
    A_mm2: W * H,
    Ix_mm4: (W * H ** 3) / 12,
    Iy_mm4: (W ** 3 * H) / 12,
    J_mm4: rectJ(W, H),
  };
})();

function resolveSection(beam: BeamDef): ResolvedSection {
  const shape = findShape(beam.params);
  if (!shape) return DEFAULT_SECTION;
  if (shape.name === 'rect') return rectSection(shape.params ?? []);
  if (shape.name === 'round') return roundSection(shape.params ?? []);
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

function rectSection(args: Param[]): ResolvedSection {
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

function roundSection(args: Param[]): ResolvedSection {
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

function explicitSection(args: Param[]): ResolvedSection {
  const Ix = findQ(args, 'Ix');
  const Iy = findQ(args, 'Iy');
  const I = findQ(args, 'I');
  const J = findQ(args, 'J');
  const A = findQ(args, 'A');
  if (J === null) return DEFAULT_SECTION;
  const IxOut = Ix !== null ? Ix : I !== null ? I : DEFAULT_SECTION.Ix_mm4;
  const IyOut = Iy !== null ? Iy : I !== null ? I : DEFAULT_SECTION.Iy_mm4;
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

// ---------- local vector helpers ----------

function addV(a: Vec3, b: Vec3): Vec3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}
function scaleV(a: Vec3, s: number): Vec3 {
  return [a[0] * s, a[1] * s, a[2] * s];
}
function dist(a: Vec3, b: Vec3): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
