import type { Beam, Frame, Problem, Vec3 } from './problem';
import type { SimError } from './simulate';

/** 3×3 row-major matrix: [m00, m01, m02, m10, m11, m12, m20, m21, m22]. */
export type Mat3 = [
  number, number, number,
  number, number, number,
  number, number, number,
];

/**
 * Deflection modes. bendIx is resisted by section Ix → deflects in beam-Y;
 * bendIy is resisted by Iy → deflects in beam-X; torsionJ twists about the
 * beam axis (resisted by J).
 */
export type Mode = 'torsionJ' | 'bendIx' | 'bendIy';

export interface Node {
  beamIx: number;
  offset_mm: number;
}

/**
 * Load node: 'force' for real loads; 'moment' appears synthetically for the
 * fixed-fixed clamp-moment reaction, paired with a synthetic clamp-force
 * reaction. `source` is 'real' for caller-supplied loads, 'clamp' for the
 * synthetic fixed-fixed reactions — the latter are popped before Compliances
 * is returned, so externally every load is 'real' / 'force'.
 */
export interface LoadNode extends Node {
  kind: 'force' | 'moment';
  source: 'real' | 'clamp';
}

export interface ComplianceEntry {
  queryIx: number;
  loadIx: number;
  beamIx: number;
  mode: Mode;
  /**
   * deflection_at_query (world) ← generalized_force_at_load (world).
   * The columns of C are interpreted per the load's `kind`: a 'force' load
   * means each column is the response to a unit world force in that axis;
   * a 'moment' load means each column is the response to a unit world moment.
   */
  C: Mat3;
}

export interface Compliances {
  /** one per beam end (chain joints) */
  queryNodes: Node[];
  /** real force loads (post-adjust, no synthetic) */
  loadNodes: LoadNode[];
  /** parallel to loadNodes */
  loadFmax_N: number[];
  /** sparse: only nonzero (q, p, b, m) */
  entries: ComplianceEntry[];
  /** Precomputed C_tot[q][p] for fast δ(d) evaluation. */
  totals: { queryIx: number; loadIx: number; C: Mat3 }[];
}

export function buildCompliances(problem: Problem): Compliances | SimError {
  const beams = problem.beams;

  // Query nodes: the caller's queries, plus — under support(both) — the root
  // beam's end. That point is a clamped joint (deflection zero by
  // construction) so it's not a reported query, but the fixed-fixed
  // compatibility solve below needs its compliance entries.
  const queryNodes: Node[] = problem.queries.map((q) => ({
    beamIx: q.beamIx,
    offset_mm: q.offset_mm,
  }));
  const fixedFixed = problem.support === 'both' && beams.length > 0;
  if (fixedFixed) {
    const rootEnd = beams[0]!.length_mm;
    if (!queryNodes.some((n) => n.beamIx === 0 && n.offset_mm === rootEnd)) {
      queryNodes.push({ beamIx: 0, offset_mm: rootEnd });
    }
  }

  // Load nodes: one per resolved load, in caller order.
  const loadNodes: LoadNode[] = [];
  const loadFmax_N: number[] = [];
  for (const ld of problem.loads) {
    loadNodes.push({ beamIx: ld.beamIx, offset_mm: ld.offset_mm, kind: 'force', source: 'real' });
    loadFmax_N.push(ld.Fmax_N);
  }

  // For support(both), append two synthetic loads at the root beam's end:
  // the reaction force and reaction moment that clamp it. Build cantilever
  // compliance with these extras, then solve a 4×4 compatibility system for
  // the chord-perpendicular reaction components (chord = root beam axis;
  // force/flexibility method).
  let clampForceLoadIx = -1;
  let clampMomentLoadIx = -1;
  if (fixedFixed) {
    const rootEnd = beams[0]!.length_mm;
    clampForceLoadIx = loadNodes.length;
    loadNodes.push({ beamIx: 0, offset_mm: rootEnd, kind: 'force', source: 'clamp' });
    loadFmax_N.push(0); // placeholder; reaction is derived, not user-supplied.
    clampMomentLoadIx = loadNodes.length;
    loadNodes.push({ beamIx: 0, offset_mm: rootEnd, kind: 'moment', source: 'clamp' });
    loadFmax_N.push(0);
  }

  // Precompute per-beam rotation matrix R_i (frame columns) and tip world pos.
  const Rs: Mat3[] = beams.map((b) => frameToR(b.frame));
  const tipWorld: Vec3[] = beams.map((b) =>
    addV(b.frame.origin_mm, scaleV(b.frame.axial, b.length_mm)),
  );

  const entries: ComplianceEntry[] = [];
  const totalsMap = new Map<string, Mat3>();

  // For fixed-fixed: rotation compliance at the *root beam's end* per load,
  // computed alongside deflection compliance in the same sweep. World-frame
  // 3×3, columns indexed by world axis of the unit input (force or moment).
  const rootEndOffset = beams.length > 0 ? beams[0]!.length_mm : 0;
  const clampQueryIx = fixedFixed
    ? queryNodes.findIndex(
        (n) => n.beamIx === 0 && n.offset_mm === rootEndOffset,
      )
    : -1;
  const collectClampRot = fixedFixed && clampQueryIx >= 0;
  const clampRot: Mat3[] = collectClampRot ? loadNodes.map(() => newMat3()) : [];

  for (let p = 0; p < loadNodes.length; p++) {
    const ln = loadNodes[p] as LoadNode;
    const k = ln.beamIx;
    const s_p = ln.offset_mm;
    const loadWorldPos = addV(
      beams[k]!.frame.origin_mm,
      scaleV(beams[k]!.frame.axial, s_p),
    );

    for (let q = 0; q < queryNodes.length; q++) {
      const qn = queryNodes[q] as Node;
      const m = qn.beamIx;
      const s_q = qn.offset_mm;
      const queryWorldPos = addV(
        beams[m]!.frame.origin_mm,
        scaleV(beams[m]!.frame.axial, s_q),
      );

      const totalC = newMat3();

      // Beams that flex from load p AND lie on the chain to the query.
      const iMax = Math.min(k, m);
      for (let i = 0; i <= iMax; i++) {
        const sect = beams[i]!.section;
        const mat = beams[i]!.material;
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
        const eTorsion = newMat3();
        const eBendIx = newMat3();
        const eBendIy = newMat3();

        for (let j = 0; j < 3; j++) {
          const dir_world: Vec3 = j === 0 ? [1, 0, 0] : j === 1 ? [0, 1, 0] : [0, 0, 1];

          // Effective (F, M) at beam i's tip in WORLD frame.
          // - 'force' load: F = unit force in world axis j; M = arm × F when
          //   the load is downstream of beam i (zero when on beam i itself).
          // - 'moment' load: F = 0; M = unit moment in world axis j (a free
          //   vector — transports unchanged from load location to beam i's
          //   tip because F = 0).
          let F_world: Vec3;
          let M_world: Vec3;
          if (ln.kind === 'force') {
            F_world = dir_world;
            if (onLoadBeam) {
              M_world = [0, 0, 0];
            } else {
              const armToLoad = subV(loadWorldPos, tip_i);
              M_world = crossV(armToLoad, F_world);
            }
          } else {
            F_world = [0, 0, 0];
            M_world = dir_world;
          }
          const F_local = matVec(R_iT, F_world);
          const M_local = matVec(R_iT, M_world);

          // Per-mode local-frame deflection + rotation at s_eval.
          const to = modeTorsion(M_local, s_load_local, s_eval_local, mat.G_MPa, sect.J_mm4);
          const bx = modeBendIx(F_local, M_local, s_load_local, s_eval_local, mat.E_MPa, sect.Ix_mm4);
          const by = modeBendIy(F_local, M_local, s_load_local, s_eval_local, mat.E_MPa, sect.Iy_mm4);

          // Transform to world and apply transport arm.
          setMatCol(eTorsion, j, worldContribution(R_i, to,  arm));
          setMatCol(eBendIx,  j, worldContribution(R_i, bx,  arm));
          setMatCol(eBendIy,  j, worldContribution(R_i, by,  arm));

          // Clamp-point rotation collection (fixed-fixed). Sum rotation across
          // the modes; rotation is a free vector, so world rotation at
          // beam i's tip is just R_i · rot_local, accumulated across beams.
          if (collectClampRot && q === clampQueryIx) {
            const rotLocalX = to.rot[0] + bx.rot[0] + by.rot[0];
            const rotLocalY = to.rot[1] + bx.rot[1] + by.rot[1];
            const rotLocalZ = to.rot[2] + bx.rot[2] + by.rot[2];
            const rW0 = R_i[0] * rotLocalX + R_i[1] * rotLocalY + R_i[2] * rotLocalZ;
            const rW1 = R_i[3] * rotLocalX + R_i[4] * rotLocalY + R_i[5] * rotLocalZ;
            const rW2 = R_i[6] * rotLocalX + R_i[7] * rotLocalY + R_i[8] * rotLocalZ;
            const T = clampRot[p]!;
            T[0 + j] = (T[0 + j] as number) + rW0;
            T[3 + j] = (T[3 + j] as number) + rW1;
            T[6 + j] = (T[6 + j] as number) + rW2;
          }
        }

        if (!isZeroMat(eTorsion)) { entries.push({ queryIx: q, loadIx: p, beamIx: i, mode: 'torsionJ', C: eTorsion }); addMat(totalC, eTorsion); }
        if (!isZeroMat(eBendIx))  { entries.push({ queryIx: q, loadIx: p, beamIx: i, mode: 'bendIx',  C: eBendIx  }); addMat(totalC, eBendIx); }
        if (!isZeroMat(eBendIy))  { entries.push({ queryIx: q, loadIx: p, beamIx: i, mode: 'bendIy',  C: eBendIy  }); addMat(totalC, eBendIy); }
      }

      totalsMap.set(`${q},${p}`, totalC);
    }
  }

  if (fixedFixed) {
    const adjusted = applyFixedFixed(
      beams, queryNodes, loadNodes, entries, totalsMap, clampRot,
      clampForceLoadIx, clampMomentLoadIx,
    );
    if (!adjusted) {
      // Degenerate chord or singular 4×4 — the support(both) problem can't be
      // solved. No silent fallback: surface it so the user fixes their input.
      return {
        kind: 'error',
        code: 'support-singular',
        message:
          'support(both): the chord is degenerate or the fixed-fixed system is singular ' +
          '— check for a zero-length root beam or a zero-stiffness section',
      };
    }
    // Adjustment succeeded; drop the two synthetic clamp loads from public API.
    loadNodes.pop(); loadFmax_N.pop();
    loadNodes.pop(); loadFmax_N.pop();
  }

  const totals = Array.from(totalsMap.entries()).map(([key, C]) => {
    const [q, p] = key.split(',').map(Number) as [number, number];
    return { queryIx: q, loadIx: p, C };
  });

  return { queryNodes, loadNodes, loadFmax_N, entries, totals };
}

// ---------- fixed-fixed (support(both)) ----------
//
// Force/flexibility method. Released structure = the full chain clamped at
// origin only (cantilever with all appendages). Unknowns at the *root beam's
// end* (= the second clamp point):
//   - reaction force  R = R_u·u + R_v·v   (chord-perpendicular plane)
//   - reaction moment M = M_u·u + M_v·v   (chord-perpendicular plane)
// The chord is the root beam's axis (origin → root beam's end). The chord-
// aligned reaction force AND the chord-aligned reaction moment are both left
// free: the first is required because beams are modeled axially rigid (no
// compliance mode along the chord), the second avoids fighting torsion that
// no real bolt-on support enforces.
//
// Compatibility: the chord-perpendicular components of the clamp point's
// deflection and rotation must both vanish. In (u, v) basis, four scalar
// equations:
//
//   u·δ_ext + (u·C_FF·u)R_u + (u·C_FF·v)R_v + (u·C_FM·u)M_u + (u·C_FM·v)M_v = 0
//   v·δ_ext + ... (similarly with v on the left)                              = 0
//   u·θ_ext + (u·C_θF·u)R_u + (u·C_θF·v)R_v + (u·C_θM·u)M_u + (u·C_θM·v)M_v = 0
//   v·θ_ext + ...                                                              = 0
//
// where (3×3 self-compliance at the clamp point):
//   C_FF = clamp-deflection ← clamp-force,  totalsMap[q_clamp][clamp_force_load]
//   C_FM = clamp-deflection ← clamp-moment, totalsMap[q_clamp][clamp_moment_load]
//   C_θF = clamp-rotation     ← clamp-force,  clampRot[clamp_force_load]
//   C_θM = clamp-rotation     ← clamp-moment, clampRot[clamp_moment_load]
//
// Solving the 4×4 once per RHS column j gives world reaction matrices
// R_world[p] (3×3, col j = reaction force for unit world force-j at load p)
// and M_world[p] (3×3, col j = reaction moment for the same). Effective
// compliance from a real load p to query q decomposes as:
//
//   C_eff[q][p]  =  C_tot[q][p]
//                  + C_tot[q][clamp_force]  · R_world[p]
//                  + C_tot[q][clamp_moment] · M_world[p]
//
// Per-(beam, mode) contributions use the same recipe on each (b, m) entry.
function applyFixedFixed(
  beams: Beam[],
  queryNodes: Node[],
  loadNodes: LoadNode[],
  entries: ComplianceEntry[],
  totalsMap: Map<string, Mat3>,
  clampRot: Mat3[],
  clampForceLoadIx: number,
  clampMomentLoadIx: number,
): boolean {
  const clampForceNode = loadNodes[clampForceLoadIx];
  const clampMomentNode = loadNodes[clampMomentLoadIx];
  if (!clampForceNode || !clampMomentNode) return false;
  const rootBeam = beams[clampForceNode.beamIx];
  if (!rootBeam) return false;
  const clampWorld: Vec3 = addV(
    rootBeam.frame.origin_mm,
    scaleV(rootBeam.frame.axial, clampForceNode.offset_mm),
  );
  const chordLen = Math.hypot(clampWorld[0], clampWorld[1], clampWorld[2]);
  if (chordLen < 1e-9) return false;
  const e: Vec3 = [clampWorld[0] / chordLen, clampWorld[1] / chordLen, clampWorld[2] / chordLen];

  // Orthonormal basis of the chord-perpendicular plane.
  const [u, v] = orthoBasisFromChord(e);

  const clampQueryIx = queryNodes.findIndex(
    (n) => n.beamIx === clampForceNode.beamIx && n.offset_mm === clampForceNode.offset_mm,
  );
  if (clampQueryIx < 0) return false;

  const C_FF = totalsMap.get(`${clampQueryIx},${clampForceLoadIx}`);
  const C_FM = totalsMap.get(`${clampQueryIx},${clampMomentLoadIx}`);
  const K_FF = clampRot[clampForceLoadIx];
  const K_FM = clampRot[clampMomentLoadIx];
  if (!C_FF || !C_FM || !K_FF || !K_FM) return false;

  // Build the 4×4 system matrix A in (u, v) basis.
  // Rows: [u·δ, v·δ, u·θ, v·θ]. Columns: [R_u, R_v, M_u, M_v].
  const A = new Array(16).fill(0) as number[];
  A[0]  = quad(u, C_FF, u); A[1]  = quad(u, C_FF, v); A[2]  = quad(u, C_FM, u); A[3]  = quad(u, C_FM, v);
  A[4]  = quad(v, C_FF, u); A[5]  = quad(v, C_FF, v); A[6]  = quad(v, C_FM, u); A[7]  = quad(v, C_FM, v);
  A[8]  = quad(u, K_FF, u); A[9]  = quad(u, K_FF, v); A[10] = quad(u, K_FM, u); A[11] = quad(u, K_FM, v);
  A[12] = quad(v, K_FF, u); A[13] = quad(v, K_FF, v); A[14] = quad(v, K_FM, u); A[15] = quad(v, K_FM, v);

  // For each real load p, solve A·X = B where B is the 4×3 negated cantilever
  // response (chord-perp components of clamp-point deflection and rotation),
  // then project X back to world to get R_world[p] and M_world[p] (3×3 each).
  const realLoadCount = loadNodes.length - 2; // last two are synthetic
  const R_world: Mat3[] = [];
  const M_world: Mat3[] = [];

  for (let p = 0; p < realLoadCount; p++) {
    const C_tp = totalsMap.get(`${clampQueryIx},${p}`) ?? newMat3();
    const K_tp = clampRot[p] ?? newMat3();
    const B = new Array(12).fill(0) as number[]; // 4 rows, 3 cols
    for (let j = 0; j < 3; j++) {
      const d0 = C_tp[0 + j] as number, d1 = C_tp[3 + j] as number, d2 = C_tp[6 + j] as number;
      const r0 = K_tp[0 + j] as number, r1 = K_tp[3 + j] as number, r2 = K_tp[6 + j] as number;
      B[0 * 3 + j] = -(u[0] * d0 + u[1] * d1 + u[2] * d2);
      B[1 * 3 + j] = -(v[0] * d0 + v[1] * d1 + v[2] * d2);
      B[2 * 3 + j] = -(u[0] * r0 + u[1] * r1 + u[2] * r2);
      B[3 * 3 + j] = -(v[0] * r0 + v[1] * r1 + v[2] * r2);
    }

    const X = solve4x3(A, B);
    if (!X) return false;

    const Rp = newMat3();
    const Mp = newMat3();
    for (let j = 0; j < 3; j++) {
      const Ru = X[0 * 3 + j] as number;
      const Rv = X[1 * 3 + j] as number;
      const Mu = X[2 * 3 + j] as number;
      const Mv = X[3 * 3 + j] as number;
      Rp[0 + j] = Ru * u[0] + Rv * v[0];
      Rp[3 + j] = Ru * u[1] + Rv * v[1];
      Rp[6 + j] = Ru * u[2] + Rv * v[2];
      Mp[0 + j] = Mu * u[0] + Mv * v[0];
      Mp[3 + j] = Mu * u[1] + Mv * v[1];
      Mp[6 + j] = Mu * u[2] + Mv * v[2];
    }
    R_world.push(Rp);
    M_world.push(Mp);
  }

  // Build (q, p, b, m) → Mat3 lookup over current entries.
  const entryMap = new Map<string, Mat3>();
  for (const en of entries) {
    entryMap.set(`${en.queryIx},${en.loadIx},${en.beamIx},${en.mode}`, en.C);
  }

  // Adjust every real-load entry: direct + via clamp-force·R + via clamp-moment·M.
  const allModes: Mode[] = ['torsionJ', 'bendIx', 'bendIy'];
  const adjusted = new Map<string, Mat3>();
  for (let q = 0; q < queryNodes.length; q++) {
    for (let p = 0; p < realLoadCount; p++) {
      for (let b = 0; b < beams.length; b++) {
        for (const mode of allModes) {
          const direct    = entryMap.get(`${q},${p},${b},${mode}`);
          const viaClampF = entryMap.get(`${q},${clampForceLoadIx},${b},${mode}`);
          const viaClampM = entryMap.get(`${q},${clampMomentLoadIx},${b},${mode}`);
          if (!direct && !viaClampF && !viaClampM) continue;
          const out = newMat3();
          if (direct)    addMat(out, direct);
          if (viaClampF) addMat(out, matMul3(viaClampF, R_world[p]!));
          if (viaClampM) addMat(out, matMul3(viaClampM, M_world[p]!));
          if (isZeroMat(out)) continue;
          adjusted.set(`${q},${p},${b},${mode}`, out);
        }
      }
    }
  }

  // Replace entries (drop synthetic-clamp entries; replace real-load entries).
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

// Gauss-Jordan elimination on [A | B] (4×4 augmented with 4×3 RHS).
// Returns X (4×3 row-major) or null if A is singular.
function solve4x3(A: number[], B: number[]): number[] | null {
  const N = 4, M = 3, W = N + M;
  const aug = new Array(N * W).fill(0) as number[];
  for (let r = 0; r < N; r++) {
    for (let c = 0; c < N; c++) aug[r * W + c] = A[r * N + c] as number;
    for (let c = 0; c < M; c++) aug[r * W + N + c] = B[r * M + c] as number;
  }
  for (let i = 0; i < N; i++) {
    let pivotRow = i;
    let pivotVal = Math.abs(aug[i * W + i] as number);
    for (let r = i + 1; r < N; r++) {
      const v = Math.abs(aug[r * W + i] as number);
      if (v > pivotVal) { pivotVal = v; pivotRow = r; }
    }
    if (!Number.isFinite(pivotVal) || pivotVal < 1e-18) return null;
    if (pivotRow !== i) {
      for (let c = 0; c < W; c++) {
        const tmp = aug[i * W + c] as number;
        aug[i * W + c] = aug[pivotRow * W + c] as number;
        aug[pivotRow * W + c] = tmp;
      }
    }
    const inv = 1 / (aug[i * W + i] as number);
    for (let c = 0; c < W; c++) aug[i * W + c] = (aug[i * W + c] as number) * inv;
    for (let r = 0; r < N; r++) {
      if (r === i) continue;
      const factor = aug[r * W + i] as number;
      if (factor === 0) continue;
      for (let c = 0; c < W; c++) {
        aug[r * W + c] = (aug[r * W + c] as number) - factor * (aug[i * W + c] as number);
      }
    }
  }
  const X = new Array(N * M).fill(0) as number[];
  for (let r = 0; r < N; r++) {
    for (let c = 0; c < M; c++) X[r * M + c] = aug[r * W + N + c] as number;
  }
  return X;
}

// ---------- per-mode formulas ----------
//
// Local frame: beam-X = frame.ex, beam-Y = frame.ey, beam-Z = frame.axial
// (along beam). Section Ix = ∫y² dA (about beam-X) → resists deflection in
// beam-Y; Iy = ∫x² dA (about beam-Y) → resists deflection in beam-X.
//
// Cantilever along +Z (fixed at base z=0), point load at offset s_load with
// local-frame force F = (F_x, F_y, F_z) and moment M = (M_x, M_y, M_z).
// Returned `defl` and `rot` are at offset s_eval, in beam-local frame.
// Modes are decoupled in Euler-Bernoulli with small deflection.
//
// Axial extension/compression is not modeled — see README scope notes.
// Chord-aligned forces produce no deflection contribution; the beam is
// treated as axially rigid.

interface ModeOut { defl: Vec3; rot: Vec3 }
const ZERO: ModeOut = { defl: [0, 0, 0], rot: [0, 0, 0] };

function modeTorsion(M: Vec3, s_load: number, s_eval: number, G_MPa: number, J_mm4: number): ModeOut {
  if (J_mm4 <= 0 || G_MPa <= 0) return ZERO;
  // θ_z(s) = M_z · min(s, s_load) / (G·J). No deflection at s_eval from
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
  // world deflection at the query = R·defl_local + (R·rot_local) × arm.
  const dW = matVec(R, mode.defl);
  const rotW = matVec(R, mode.rot);
  const transport = crossV(rotW, arm_world);
  return [dW[0] + transport[0], dW[1] + transport[1], dW[2] + transport[2]];
}

// ---------- helpers ----------

function frameToR(f: Frame): Mat3 {
  // R = [ex | ey | axial] as columns (row-major storage).
  return [
    f.ex[0], f.ey[0], f.axial[0],
    f.ex[1], f.ey[1], f.axial[1],
    f.ex[2], f.ey[2], f.axial[2],
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
