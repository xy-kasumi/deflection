import type { Diagnostic } from '../dsl/diagnostics';
import type { Attachment, Structure } from '../dsl/parse';
import { KGF_TO_N, MATERIALS, type MaterialId } from '../state';
import type { BeamNode, Frame, Vec3 } from '../walker';
import { resolveSection, type Section } from './section';

// 3×3 row-major matrix: [m00, m01, m02, m10, m11, m12, m20, m21, m22].
export type Mat3 = [
  number, number, number,
  number, number, number,
  number, number, number,
];

export type Mode = 'axial' | 'torsion' | 'bendIx' | 'bendIy';

export interface Node {
  beamIx: number;
  offset_mm: number;
}

export interface ComplianceEntry {
  queryIx: number;
  loadIx: number;
  beamIx: number;
  mode: Mode;
  C: Mat3; // displacement_at_query (world) ←  force_at_load (world)
}

export interface Compliances {
  queryNodes: Node[];          // one per beam end (chain joints)
  loadNodes: Node[];           // one per force attachment
  loadFmax_N: number[];        // parallel to loadNodes
  entries: ComplianceEntry[];  // sparse: only nonzero (q, p, b, m)
  // Precomputed C_tot[q][p] for fast δ(d) evaluation.
  totals: { queryIx: number; loadIx: number; C: Mat3 }[];
}

export function buildCompliances(
  beams: BeamNode[],
  structure: Structure,
): { compliances: Compliances; diagnostics: Diagnostic[] } {
  const diagnostics: Diagnostic[] = [];

  // Per-beam section + material.
  const sections = beams.map((b) => resolveSection(b.def));
  const matIds: MaterialId[] = beams.map((b) =>
    isKnownMaterial(b.material) ? (b.material as MaterialId) : 'plastic',
  );
  const mats = matIds.map((id) => MATERIALS[id]);

  // Query nodes: one per beam end (chain joints in source order).
  const queryNodes: Node[] = beams.map((b, i) => ({
    beamIx: i,
    offset_mm: b.length_mm,
  }));

  // Load nodes: one per force attachment (in chain order).
  const loadNodes: Node[] = [];
  const loadFmax_N: number[] = [];
  for (let i = 0; i < beams.length; i++) {
    const b = beams[i] as BeamNode;
    for (const att of b.attachmentOffsets) {
      if (att.def.name !== 'force') continue;
      const F = forceMagnitudeN(att.def);
      if (F <= 0) continue;
      loadNodes.push({ beamIx: i, offset_mm: att.local_mm });
      loadFmax_N.push(F);
    }
  }

  // Precompute per-beam rotation matrix R_i (frame columns) and tip world pos.
  const Rs: Mat3[] = beams.map((b) => frameToR(b.startFrame));
  const tipWorld: Vec3[] = beams.map((b) =>
    addV(b.startFrame.origin, scaleV(b.startFrame.fwd, b.length_mm)),
  );

  const entries: ComplianceEntry[] = [];
  const totalsMap = new Map<string, Mat3>();

  for (let p = 0; p < loadNodes.length; p++) {
    const ln = loadNodes[p] as Node;
    const k = ln.beamIx;
    const s_p = ln.offset_mm;
    const loadWorldPos = addV(
      beams[k]!.startFrame.origin,
      scaleV(beams[k]!.startFrame.fwd, s_p),
    );

    for (let q = 0; q < queryNodes.length; q++) {
      const qn = queryNodes[q] as Node;
      const m = qn.beamIx;
      const s_q = qn.offset_mm;
      const queryWorldPos = addV(
        beams[m]!.startFrame.origin,
        scaleV(beams[m]!.startFrame.fwd, s_q),
      );

      const totalC = newMat3();

      // Beams that flex from load p AND lie on the chain to the query.
      const iMax = Math.min(k, m);
      for (let i = 0; i <= iMax; i++) {
        const sect = sections[i]!;
        const mat = mats[i]!;
        const L_i = beams[i]!.length_mm;
        const R_i = Rs[i]!;
        const R_iT = transposeM(R_i);
        const tip_i = tipWorld[i]!;

        const onLoadBeam = i === k;
        const onQueryBeam = i === m;

        const s_load_local = onLoadBeam ? s_p : L_i;
        const s_eval_local = onQueryBeam ? s_q : L_i;

        // Where the contribution is sensed in world frame (for transport arm).
        // - if onQueryBeam: at queryWorldPos (no transport arm)
        // - else (i < m): at beam i's tip; arm to query = queryWorldPos - tip_i.
        const arm = onQueryBeam ? [0, 0, 0] as Vec3 : subV(queryWorldPos, tip_i);

        // Three columns of the per-(q, p, i, mode) entry: one per world force axis.
        const eAxial = newMat3();
        const eTorsion = newMat3();
        const eBendIx = newMat3();
        const eBendIy = newMat3();

        for (let j = 0; j < 3; j++) {
          const F_world: Vec3 = j === 0 ? [1, 0, 0] : j === 1 ? [0, 1, 0] : [0, 0, 1];
          // Local-frame load on beam i.
          const F_local = matVec(R_iT, F_world);
          let M_local: Vec3 = [0, 0, 0];
          if (!onLoadBeam) {
            // Effective tip load on beam i: F at world location of load.
            // Moment about beam i's tip = (loadWorldPos - tip_i) × F_world.
            const armToLoad = subV(loadWorldPos, tip_i);
            const M_world = crossV(armToLoad, F_world);
            M_local = matVec(R_iT, M_world);
          }

          // Per-mode local-frame deflection + rotation at s_eval.
          const ax = modeAxial(F_local, s_load_local, s_eval_local, mat.E_MPa, sect.A_mm2);
          const to = modeTorsion(M_local, s_load_local, s_eval_local, mat.G_MPa, sect.J_mm4);
          const bx = modeBendIx(F_local, M_local, s_load_local, s_eval_local, mat.E_MPa, sect.Ix_mm4);
          const by = modeBendIy(F_local, M_local, s_load_local, s_eval_local, mat.E_MPa, sect.Iy_mm4);

          // Transform to world and apply transport arm.
          setMatCol(eAxial,   j, worldContribution(R_i, ax,  arm));
          setMatCol(eTorsion, j, worldContribution(R_i, to,  arm));
          setMatCol(eBendIx,  j, worldContribution(R_i, bx,  arm));
          setMatCol(eBendIy,  j, worldContribution(R_i, by,  arm));
        }

        if (!isZeroMat(eAxial))   { entries.push({ queryIx: q, loadIx: p, beamIx: i, mode: 'axial',   C: eAxial   }); addMat(totalC, eAxial); }
        if (!isZeroMat(eTorsion)) { entries.push({ queryIx: q, loadIx: p, beamIx: i, mode: 'torsion', C: eTorsion }); addMat(totalC, eTorsion); }
        if (!isZeroMat(eBendIx))  { entries.push({ queryIx: q, loadIx: p, beamIx: i, mode: 'bendIx',  C: eBendIx  }); addMat(totalC, eBendIx); }
        if (!isZeroMat(eBendIy))  { entries.push({ queryIx: q, loadIx: p, beamIx: i, mode: 'bendIy',  C: eBendIy  }); addMat(totalC, eBendIy); }
      }

      totalsMap.set(`${q},${p}`, totalC);
    }
  }

  const totals = Array.from(totalsMap.entries()).map(([key, C]) => {
    const [q, p] = key.split(',').map(Number) as [number, number];
    return { queryIx: q, loadIx: p, C };
  });

  // Optional: warn unsupported support config (single-beam chain w/ both is OK,
  // multi-beam chain w/ both is deferred; main path treats as cantilever either way).
  for (const env of structure.envs) {
    if (env.name === 'support') {
      const arg = env.params[0];
      const argName = arg && arg.kind === 'ident' ? arg.name : '';
      if (argName === 'both' && beams.length > 1) {
        diagnostics.push({
          severity: 'warning',
          message: 'support(both) on a multi-beam chain is not yet supported — falling back to support(single)',
          span: env.span,
        });
      }
    }
  }

  return {
    compliances: {
      queryNodes,
      loadNodes,
      loadFmax_N,
      entries,
      totals,
    },
    diagnostics,
  };
}

// ---------- per-mode formulas ----------
//
// Local frame: beam-X = walker.right, beam-Y = walker.up, beam-Z = walker.fwd
// (along beam). Section Ix = ∫y² dA (about beam-X) → resists deflection in
// beam-Y; Iy = ∫x² dA (about beam-Y) → resists deflection in beam-X.
//
// Cantilever along +Z (fixed at base z=0), point load at offset s_load with
// local-frame force F = (F_x, F_y, F_z) and moment M = (M_x, M_y, M_z).
// Returned `defl` and `rot` are at offset s_eval, in beam-local frame.
// Modes are decoupled in Euler-Bernoulli with small deflection.

interface ModeOut { defl: Vec3; rot: Vec3 }
const ZERO: ModeOut = { defl: [0, 0, 0], rot: [0, 0, 0] };

function modeAxial(F: Vec3, s_load: number, s_eval: number, E_MPa: number, A_mm2: number | null): ModeOut {
  if (A_mm2 === null || A_mm2 <= 0 || E_MPa <= 0) return ZERO;
  // u_z(s) = F_z · min(s, s_load) / (E·A)
  const u_z = F[2] * Math.min(s_eval, s_load) / (E_MPa * A_mm2);
  return { defl: [0, 0, u_z], rot: [0, 0, 0] };
}

function modeTorsion(M: Vec3, s_load: number, s_eval: number, G_MPa: number, J_mm4: number): ModeOut {
  if (J_mm4 <= 0 || G_MPa <= 0) return ZERO;
  // θ_z(s) = M_z · min(s, s_load) / (G·J). No displacement at s_eval from
  // torsion alone (rotation propagates as transport for downstream points).
  const theta_z = M[2] * Math.min(s_eval, s_load) / (G_MPa * J_mm4);
  return { defl: [0, 0, 0], rot: [0, 0, theta_z] };
}

function modeBendIx(F: Vec3, M: Vec3, s_load: number, s_eval: number, E_MPa: number, Ix_mm4: number): ModeOut {
  if (Ix_mm4 <= 0 || E_MPa <= 0) return ZERO;
  const EI = E_MPa * Ix_mm4;
  const Fy = F[1];
  const Mx = M[0];
  let u_y: number;
  let du_y_ds: number;
  if (s_eval <= s_load) {
    u_y     = (Fy * s_eval * s_eval * (3 * s_load - s_eval) / 6 + Mx * s_eval * s_eval / 2) / EI;
    du_y_ds = (Fy * s_eval * (s_load - s_eval / 2)            + Mx * s_eval)                / EI;
  } else {
    const u_at  = (Fy * s_load * s_load * s_load / 3 + Mx * s_load * s_load / 2) / EI;
    const du_at = (Fy * s_load * s_load / 2          + Mx * s_load)              / EI;
    u_y     = u_at + du_at * (s_eval - s_load);
    du_y_ds = du_at;
  }
  // θ_x = -du_y/ds (rotation about +X tilts +Z toward -Y).
  return { defl: [0, u_y, 0], rot: [-du_y_ds, 0, 0] };
}

function modeBendIy(F: Vec3, M: Vec3, s_load: number, s_eval: number, E_MPa: number, Iy_mm4: number): ModeOut {
  if (Iy_mm4 <= 0 || E_MPa <= 0) return ZERO;
  const EI = E_MPa * Iy_mm4;
  const Fx = F[0];
  const My = M[1];
  let u_x: number;
  let du_x_ds: number;
  if (s_eval <= s_load) {
    u_x     = (Fx * s_eval * s_eval * (3 * s_load - s_eval) / 6 - My * s_eval * s_eval / 2) / EI;
    du_x_ds = (Fx * s_eval * (s_load - s_eval / 2)            - My * s_eval)                / EI;
  } else {
    const u_at  = (Fx * s_load * s_load * s_load / 3 - My * s_load * s_load / 2) / EI;
    const du_at = (Fx * s_load * s_load / 2          - My * s_load)              / EI;
    u_x     = u_at + du_at * (s_eval - s_load);
    du_x_ds = du_at;
  }
  // θ_y = du_x/ds (rotation about +Y tilts +Z toward +X).
  return { defl: [u_x, 0, 0], rot: [0, du_x_ds, 0] };
}

// ---------- world transport ----------

function worldContribution(R: Mat3, mode: ModeOut, arm_world: Vec3): Vec3 {
  // world displacement at the query = R·defl_local + (R·rot_local) × arm.
  const dW = matVec(R, mode.defl);
  const rotW = matVec(R, mode.rot);
  const transport = crossV(rotW, arm_world);
  return [dW[0] + transport[0], dW[1] + transport[1], dW[2] + transport[2]];
}

// ---------- helpers ----------

function isKnownMaterial(s: string | undefined): boolean {
  return s === 'plastic' || s === 'aluminum' || s === 'steel';
}

function forceMagnitudeN(att: Attachment): number {
  if (att.params.length !== 1) return 0;
  const p = att.params[0]!;
  if (p.kind !== 'quantity') return 0;
  const v = p.quantity.value;
  if (!Number.isFinite(v) || v <= 0) return 0;
  const unit = p.quantity.unit ?? 'kgf';
  if (unit === 'N')   return v;
  if (unit === 'kgf') return v * KGF_TO_N;
  return v * KGF_TO_N;
}

function frameToR(f: Frame): Mat3 {
  // R = [right | up | fwd] as columns (row-major storage).
  return [
    f.right[0], f.up[0], f.fwd[0],
    f.right[1], f.up[1], f.fwd[1],
    f.right[2], f.up[2], f.fwd[2],
  ];
}

function transposeM(M: Mat3): Mat3 {
  return [M[0], M[3], M[6], M[1], M[4], M[7], M[2], M[5], M[8]];
}

function matVec(M: Mat3, v: Vec3): Vec3 {
  return [
    M[0] * v[0] + M[1] * v[1] + M[2] * v[2],
    M[3] * v[0] + M[4] * v[1] + M[5] * v[2],
    M[6] * v[0] + M[7] * v[1] + M[8] * v[2],
  ];
}

function newMat3(): Mat3 {
  return [0, 0, 0, 0, 0, 0, 0, 0, 0];
}

function setMatCol(M: Mat3, col: number, v: Vec3): void {
  M[col + 0] = v[0];
  M[col + 3] = v[1];
  M[col + 6] = v[2];
}

function addMat(into: Mat3, m: Mat3): void {
  for (let i = 0; i < 9; i++) (into[i] as number) += m[i] as number;
}

function isZeroMat(m: Mat3): boolean {
  for (let i = 0; i < 9; i++) if ((m[i] as number) !== 0) return false;
  return true;
}

function addV(a: Vec3, b: Vec3): Vec3 { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
function subV(a: Vec3, b: Vec3): Vec3 { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
function scaleV(a: Vec3, s: number): Vec3 { return [a[0] * s, a[1] * s, a[2] * s]; }
function crossV(a: Vec3, b: Vec3): Vec3 {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}
