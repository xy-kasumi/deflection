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

  // Query nodes: candidate is each beam's beginning *and* end, deduped by
  // world position so a beam-end that coincides with the next beam's beginning
  // is counted once. We drop the origin (always fixed by support(single)),
  // but keep the tip even for support(both) because the pin-roller solve below
  // needs its compliance entries — run.ts will hide the constrained tip from
  // the public node list.
  const queryNodes: Node[] = buildQueryNodes(beams);

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

  // For support(both), append a synthetic load node at the chain tip — the
  // unknown reaction force the tip support contributes. We build cantilever
  // compliance with this extra load, then solve for the reaction in a
  // perpendicular-to-chord plane (force/flexibility method).
  const supportKind = getSupportKind(structure);
  const pinRoller = supportKind === 'both' && beams.length > 0;
  let tipLoadIx = -1;
  if (pinRoller) {
    const last = beams.length - 1;
    tipLoadIx = loadNodes.length;
    loadNodes.push({ beamIx: last, offset_mm: beams[last]!.length_mm });
    loadFmax_N.push(0); // placeholder; reaction is derived, not user-supplied.
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

  if (pinRoller) {
    const adjusted = applyPinRoller(
      beams, queryNodes, loadNodes, entries, totalsMap, tipLoadIx,
    );
    if (!adjusted) {
      // Couldn't apply (degenerate chord etc.) — drop the synthesized tip load
      // and fall back to cantilever. A diagnostic is added below.
      loadNodes.pop();
      loadFmax_N.pop();
      for (let i = entries.length - 1; i >= 0; i--) {
        if (entries[i]!.loadIx === tipLoadIx) entries.splice(i, 1);
      }
      totalsMap.forEach((_, key) => {
        if (Number(key.split(',')[1]) === tipLoadIx) totalsMap.delete(key);
      });
      diagnostics.push(supportFallbackDiag(structure));
    } else {
      // Adjustment succeeded; drop the synthesized tip load from public API.
      loadNodes.pop();
      loadFmax_N.pop();
    }
  }

  const totals = Array.from(totalsMap.entries()).map(([key, C]) => {
    const [q, p] = key.split(',').map(Number) as [number, number];
    return { queryIx: q, loadIx: p, C };
  });

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

// ---------- pin-roller (support(both)) ----------
//
// Force/flexibility method: the released structure is the cantilever already
// built above; the unknown is the tip's reaction force R, constrained to the
// plane perpendicular to the root→tip chord (so the chord component is free —
// otherwise the chain is over-constrained against axial elongation in a way
// no real bolt-on support enforces).
//
// Solve for R from the compatibility condition that the chord-perpendicular
// component of the tip's displacement is zero:
//
//   P · u_tip_ext  +  P · C_RR · R  =  0
//   R  =  −P · (P · C_RR · P)⁺ · P · u_tip_ext
//      =  M_neg · u_tip_ext
//
// where C_RR = C_tot[q_tip][tip_load], P = I − e·eᵀ. Then the effective
// compliance from a real load p to query q becomes
//
//   C_eff[q][p]  =  C_tot[q][p]  +  C_tot[q][tip] · M_p,
//   M_p          =  M_neg · C_tot[q_tip][p].
//
// Per-(beam, mode) attribution decomposes the same way: each beam's effective
// entry gets the indirect-via-tip term added on top of its direct entry.
//
// Caveat: the root is still treated as fully clamped, so this is a
// propped-cantilever model, not classical pinned-pinned. Good enough as a
// rigid-bolted support model; not the textbook simply-supported beam.
function applyPinRoller(
  beams: BeamNode[],
  queryNodes: Node[],
  loadNodes: Node[],
  entries: ComplianceEntry[],
  totalsMap: Map<string, Mat3>,
  tipLoadIx: number,
): boolean {
  const tipNode = loadNodes[tipLoadIx];
  if (!tipNode) return false;
  const last = beams[tipNode.beamIx];
  if (!last) return false;
  const tipWorld: Vec3 = addV(
    last.startFrame.origin,
    scaleV(last.startFrame.fwd, tipNode.offset_mm),
  );
  const chordLen = Math.hypot(tipWorld[0], tipWorld[1], tipWorld[2]);
  if (chordLen < 1e-9) return false;
  const e: Vec3 = [tipWorld[0] / chordLen, tipWorld[1] / chordLen, tipWorld[2] / chordLen];

  // Orthonormal basis of the chord-perpendicular plane.
  const [u, v] = orthoBasisFromChord(e);

  const tipQueryIx = queryNodes.findIndex(
    (n) => n.beamIx === tipNode.beamIx && n.offset_mm === tipNode.offset_mm,
  );
  if (tipQueryIx < 0) return false;
  const C_RR = totalsMap.get(`${tipQueryIx},${tipLoadIx}`);
  if (!C_RR) return false;

  // 2×2 in (u, v) basis.
  const Cuu = quad(u, C_RR, u);
  const Cuv = quad(u, C_RR, v);
  const Cvu = quad(v, C_RR, u);
  const Cvv = quad(v, C_RR, v);
  const det = Cuu * Cvv - Cuv * Cvu;
  if (!Number.isFinite(det) || Math.abs(det) < 1e-18) return false;
  const Iuu =  Cvv / det;
  const Iuv = -Cuv / det;
  const Ivu = -Cvu / det;
  const Ivv =  Cuu / det;

  // M_neg = −(Iuu u uᵀ + Iuv u vᵀ + Ivu v uᵀ + Ivv v vᵀ).
  const M_neg = newMat3();
  addOuter(M_neg, u, u, -Iuu);
  addOuter(M_neg, u, v, -Iuv);
  addOuter(M_neg, v, u, -Ivu);
  addOuter(M_neg, v, v, -Ivv);

  // M_p[p] = M_neg · C_tot[q_tip][p] for each real load p.
  const realLoadCount = loadNodes.length - 1; // tip is the last entry.
  const M_p: Mat3[] = [];
  for (let p = 0; p < realLoadCount; p++) {
    const C_tp = totalsMap.get(`${tipQueryIx},${p}`) ?? newMat3();
    M_p.push(matMul3(M_neg, C_tp));
  }

  // Build (q, p, b, m) → Mat3 lookup over current entries.
  const entryMap = new Map<string, Mat3>();
  for (const en of entries) {
    entryMap.set(`${en.queryIx},${en.loadIx},${en.beamIx},${en.mode}`, en.C);
  }

  // Adjust every real-load entry by the via-tip term.
  const allModes: Mode[] = ['axial', 'torsion', 'bendIx', 'bendIy'];
  const adjusted = new Map<string, Mat3>();
  for (let q = 0; q < queryNodes.length; q++) {
    for (let p = 0; p < realLoadCount; p++) {
      for (let b = 0; b < beams.length; b++) {
        for (const mode of allModes) {
          const direct = entryMap.get(`${q},${p},${b},${mode}`);
          const viaTip = entryMap.get(`${q},${tipLoadIx},${b},${mode}`);
          if (!direct && !viaTip) continue;
          const out = newMat3();
          if (direct) addMat(out, direct);
          if (viaTip) addMat(out, matMul3(viaTip, M_p[p]!));
          if (isZeroMat(out)) continue;
          adjusted.set(`${q},${p},${b},${mode}`, out);
        }
      }
    }
  }

  // Replace entries (drop tip-load entries; replace real-load entries).
  entries.length = 0;
  for (const [key, C] of adjusted) {
    const [qStr, pStr, bStr, mStr] = key.split(',');
    entries.push({
      queryIx: Number(qStr),
      loadIx: Number(pStr),
      beamIx: Number(bStr),
      mode: mStr as Mode,
      C,
    });
  }

  // Rebuild totals from adjusted entries.
  totalsMap.clear();
  for (const en of entries) {
    const key = `${en.queryIx},${en.loadIx}`;
    let tot = totalsMap.get(key);
    if (!tot) { tot = newMat3(); totalsMap.set(key, tot); }
    addMat(tot, en.C);
  }
  return true;
}

export function getSupportKind(structure: Structure): 'single' | 'both' | undefined {
  for (const env of structure.envs) {
    if (env.name !== 'support') continue;
    const arg = env.params[0];
    if (arg && arg.kind === 'ident' && (arg.name === 'single' || arg.name === 'both')) {
      return arg.name;
    }
  }
  return undefined;
}

const QUERY_POS_EPS = 1e-6;
function buildQueryNodes(beams: BeamNode[]): Node[] {
  const out: Node[] = [];
  const seen: Vec3[] = [];
  for (let i = 0; i < beams.length; i++) {
    const b = beams[i]!;
    const endpoints: { offset_mm: number; world: Vec3 }[] = [
      { offset_mm: 0, world: b.startFrame.origin },
      { offset_mm: b.length_mm, world: addV(b.startFrame.origin, scaleV(b.startFrame.fwd, b.length_mm)) },
    ];
    for (const e of endpoints) {
      if (Math.hypot(e.world[0], e.world[1], e.world[2]) < QUERY_POS_EPS) continue; // origin
      let dup = false;
      for (const sp of seen) {
        if (Math.abs(e.world[0] - sp[0]) < QUERY_POS_EPS
         && Math.abs(e.world[1] - sp[1]) < QUERY_POS_EPS
         && Math.abs(e.world[2] - sp[2]) < QUERY_POS_EPS) { dup = true; break; }
      }
      if (dup) continue;
      out.push({ beamIx: i, offset_mm: e.offset_mm });
      seen.push(e.world);
    }
  }
  return out;
}

function supportFallbackDiag(structure: Structure): Diagnostic {
  const envSpan = structure.envs.find((e) => e.name === 'support')?.span ?? { start: 0, end: 0 };
  return {
    severity: 'warning',
    message: 'support(both): chord is degenerate or compliance is singular — falling back to support(single)',
    span: envSpan,
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

function matMul3(A: Mat3, B: Mat3): Mat3 {
  const C = newMat3();
  for (let r = 0; r < 3; r++) {
    for (let c = 0; c < 3; c++) {
      let s = 0;
      for (let k = 0; k < 3; k++) s += (A[r * 3 + k] as number) * (B[k * 3 + c] as number);
      C[r * 3 + c] = s;
    }
  }
  return C;
}

function quad(u: Vec3, M: Mat3, v: Vec3): number {
  // uᵀ M v
  return u[0] * (M[0] * v[0] + M[1] * v[1] + M[2] * v[2])
       + u[1] * (M[3] * v[0] + M[4] * v[1] + M[5] * v[2])
       + u[2] * (M[6] * v[0] + M[7] * v[1] + M[8] * v[2]);
}

function addOuter(into: Mat3, a: Vec3, b: Vec3, s: number): void {
  into[0] += s * a[0] * b[0]; into[1] += s * a[0] * b[1]; into[2] += s * a[0] * b[2];
  into[3] += s * a[1] * b[0]; into[4] += s * a[1] * b[1]; into[5] += s * a[1] * b[2];
  into[6] += s * a[2] * b[0]; into[7] += s * a[2] * b[1]; into[8] += s * a[2] * b[2];
}

// Returns two orthonormal vectors spanning the plane perpendicular to `e`.
function orthoBasisFromChord(e: Vec3): [Vec3, Vec3] {
  const ax: Vec3 =
    Math.abs(e[0]) <= Math.abs(e[1]) && Math.abs(e[0]) <= Math.abs(e[2])
      ? [1, 0, 0]
      : Math.abs(e[1]) <= Math.abs(e[2])
        ? [0, 1, 0]
        : [0, 0, 1];
  const c1 = crossV(e, ax);
  const c1len = Math.hypot(c1[0], c1[1], c1[2]);
  const u: Vec3 = [c1[0] / c1len, c1[1] / c1len, c1[2] / c1len];
  const v = crossV(e, u);
  return [u, v];
}
