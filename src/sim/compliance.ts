import type { Frame, Problem } from './problem';
import type { Vec3, Mat3 } from './math';
import type { SimError } from './simulate';
import { buildSegmentation, type Segment } from './segment';

/**
 * Per-beam deflection sub-modes, each rank-1 at any query: the (deflection
 * at query) under unit load at p factors as (scalar linear in F) × (fixed
 * world-frame direction), so each entry's compliance Mat3 has rank ≤ 1.
 *
 *   twist        — torsion (resisted by J); rotation × arm only.
 *   bendIxTrans  — bendIx translation: u_y · e_y^i  (defl in beam-Y, world R·e_y).
 *   bendIxRot    — bendIx rotation transport: θ_x · (e_x^i × arm).
 *   bendIyTrans  — bendIy translation: u_x · e_x^i.
 *   bendIyRot    — bendIy rotation transport: θ_y · (e_y^i × arm).
 *
 * For on-beam queries (i = m), arm = 0, so the three *Rot modes contribute
 * zero to translation. The rotation itself is still present and shows up in
 * δθ (see `rotation` on DeflectionQueryResult).
 */
export type Mode =
  | 'twist'
  | 'bendIxTrans'
  | 'bendIxRot'
  | 'bendIyTrans'
  | 'bendIyRot';

interface ComplianceEntry {
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
  /** F_max for each real load, in caller order. */
  loadFmax_N: number[];
  /** sparse: only nonzero (q, p, b, m) */
  entries: ComplianceEntry[];
  /** Precomputed C_tot[q][p] for fast δ(d) evaluation. */
  totals: { queryIx: number; loadIx: number; C: Mat3 }[];
  /**
   * Rotation compliance C_rot[q][p]: world-frame 3×3 mapping a unit world
   * force at load p to a linearized world-frame rotation vector at query q.
   * Used to build a parallel ConvexEnvelope for δθ_q (rotation worst-case),
   * exposed alongside δ_q. Not decomposed per (b, m) — δθ is reported only
   * as a headline; bend-rotation contributions to *translation* still show
   * up in the per-(b, m) decomposition via the rotation × arm path.
   */
  rotationTotals: { queryIx: number; loadIx: number; C: Mat3 }[];
}

export function buildCompliances(problem: Problem): Compliances | SimError {
  const seg = buildSegmentation(problem);
  if ('kind' in seg) return seg;

  if (seg.segments.length === 0) {
    return { loadFmax_N: [], entries: [], totals: [], rotationTotals: [] };
  }

  const beamCount = problem.beams.length;
  const fixedFixed = problem.support === 'both';

  // ---------- Load and query tables (real + clamp synthetics) ----------
  // Loads are listed in caller order (matches segLoad.loadIx ordering); fixed-
  // fixed appends two synthetics at the root beam's end node — a force and a
  // moment — for the compatibility solve. They're dropped before public output.
  interface LoadEntry { nodeIx: number; kind: 'force' | 'moment' }
  const loadEntries: LoadEntry[] = [];
  const loadFmax_N: number[] = [];
  const segLoadByIx = new Map<number, typeof seg.loads[number]>();
  for (const sl of seg.loads) segLoadByIx.set(sl.loadIx, sl);
  for (let ix = 0; ix < problem.loads.length; ix++) {
    const sl = segLoadByIx.get(ix)!;
    loadEntries.push({ nodeIx: sl.nodeIx, kind: 'force' });
    loadFmax_N.push(sl.Fmax_N);
  }
  const realLoadCount = problem.loads.length;

  let clampForceLoadIx = -1;
  let clampMomentLoadIx = -1;
  const clampNodeIx = fixedFixed ? seg.beamEndNode[0]! : -1;
  if (fixedFixed) {
    clampForceLoadIx = loadEntries.length;
    loadEntries.push({ nodeIx: clampNodeIx, kind: 'force' });
    loadFmax_N.push(0);
    clampMomentLoadIx = loadEntries.length;
    loadEntries.push({ nodeIx: clampNodeIx, kind: 'moment' });
    loadFmax_N.push(0);
  }

  const queryNodeIxs: number[] = new Array(problem.queries.length);
  const segQueryByIx = new Map<number, typeof seg.queries[number]>();
  for (const sq of seg.queries) segQueryByIx.set(sq.queryIx, sq);
  for (let ix = 0; ix < problem.queries.length; ix++) {
    queryNodeIxs[ix] = segQueryByIx.get(ix)!.nodeIx;
  }
  let clampQueryIx = -1;
  if (fixedFixed) {
    const existing = queryNodeIxs.indexOf(clampNodeIx);
    if (existing >= 0) clampQueryIx = existing;
    else {
      clampQueryIx = queryNodeIxs.length;
      queryNodeIxs.push(clampNodeIx);
    }
  }

  // ---------- Tree topology helpers ----------
  // parentSeg[n] = the segment whose tip is node n. Undefined at the root.
  // The chain from root to any node is the unique path obtained by walking
  // parentSeg upward. For a (load_node, query_node) pair, the segments that
  // *flex* are the common prefix of the two chains — i.e. path root → LCA.
  // Segments past the LCA on the load side carry the load internally but
  // their motion does not propagate up to the query side (it's downstream).
  const parentSeg = new Array<Segment | undefined>(seg.nodes.length).fill(undefined);
  for (const s of seg.segments) parentSeg[s.tipNodeIx] = s;

  const segR = new Map<Segment, { R: Mat3; RT: Mat3 }>();
  for (const s of seg.segments) {
    const R = frameToR(s.frame);
    segR.set(s, { R, RT: transposeM(R) });
  }

  function chainTo(nodeIx: number): Segment[] {
    const out: Segment[] = [];
    let cur = nodeIx;
    while (parentSeg[cur]) {
      out.push(parentSeg[cur]!);
      cur = parentSeg[cur]!.baseNodeIx;
    }
    return out.reverse();
  }
  const chainPerLoad = loadEntries.map((l) => chainTo(l.nodeIx));
  const chainPerQuery = queryNodeIxs.map((nodeIx) => chainTo(nodeIx));

  // ---------- Per-(load, query) compliance walk ----------
  // Per-(q, p, b, m) accumulator. One user beam can span several segments
  // (when attachments/loads/queries split it), so multiple segment-level
  // contributions get summed into the same (b, m) bucket.
  const entryMap = new Map<string, Mat3>();
  const totalsMap = new Map<string, Mat3>();
  const rotMap = new Map<string, Mat3>();

  for (let p = 0; p < loadEntries.length; p++) {
    const ln = loadEntries[p]!;
    const loadWorldPos = seg.nodes[ln.nodeIx]!.pos_mm;
    const chainL = chainPerLoad[p]!;

    for (let q = 0; q < queryNodeIxs.length; q++) {
      const queryNodeIx = queryNodeIxs[q]!;
      const queryWorldPos = seg.nodes[queryNodeIx]!.pos_mm;
      const chainQ = chainPerQuery[q]!;

      // Common prefix = path root → LCA(load_node, query_node). Reference
      // equality is sufficient: both chains share the same `Segment`
      // instances out of `seg.segments`.
      let commonLen = 0;
      const lenMin = Math.min(chainL.length, chainQ.length);
      while (commonLen < lenMin && chainL[commonLen] === chainQ[commonLen]) commonLen++;

      const totalC = newMat3();

      for (let ci = 0; ci < commonLen; ci++) {
        const s = chainL[ci]!;
        const { R, RT } = segR.get(s)!;
        const tipPos = seg.nodes[s.tipNodeIx]!.pos_mm;
        const L_seg = s.length_mm;

        // Every load and query lives at a segment endpoint by construction,
        // so the cantilever's "load" and "eval" both sit at this segment's
        // tip: s_load = s_eval = L_seg.
        const armToQuery: Vec3 = (queryNodeIx === s.tipNodeIx)
          ? [0, 0, 0]
          : subV(queryWorldPos, tipPos);

        const eTwist       = newMat3();
        const eBendIxTrans = newMat3();
        const eBendIxRot   = newMat3();
        const eBendIyTrans = newMat3();
        const eBendIyRot   = newMat3();

        for (let j = 0; j < 3; j++) {
          const dir_world: Vec3 = j === 0 ? [1, 0, 0] : j === 1 ? [0, 1, 0] : [0, 0, 1];

          // (F, M) at the segment's tip in WORLD frame.
          // - 'force' load: F = unit force in world axis j; M = arm × F when
          //   the load is downstream of this tip (zero when load IS at this
          //   tip — i.e. the segment hosts the load directly).
          // - 'moment' load: F = 0; M = unit moment in world axis j (free
          //   vector — transports unchanged because F = 0).
          let F_world: Vec3;
          let M_world: Vec3;
          if (ln.kind === 'force') {
            F_world = dir_world;
            if (ln.nodeIx === s.tipNodeIx) {
              M_world = [0, 0, 0];
            } else {
              const armToLoad = subV(loadWorldPos, tipPos);
              M_world = crossV(armToLoad, F_world);
            }
          } else {
            F_world = [0, 0, 0];
            M_world = dir_world;
          }
          const F_local = matVec(RT, F_world);
          const M_local = matVec(RT, M_world);

          const to = modeTorsion(M_local, L_seg, s.material.G_MPa, s.section.J_mm4);
          const bx = modeBendIx(F_local, M_local, L_seg, s.material.E_MPa, s.section.Ix_mm4);
          const by = modeBendIy(F_local, M_local, L_seg, s.material.E_MPa, s.section.Iy_mm4);

          setMatCol(eTwist,       j, transportOnly(R, to.rot, armToQuery));
          setMatCol(eBendIxTrans, j, translationOnly(R, bx.defl));
          setMatCol(eBendIxRot,   j, transportOnly(R, bx.rot, armToQuery));
          setMatCol(eBendIyTrans, j, translationOnly(R, by.defl));
          setMatCol(eBendIyRot,   j, transportOnly(R, by.rot, armToQuery));

          // Rotation accumulation per (q, p). Rotation is a free vector, so
          // world rotation at the segment tip is R · rot_local; accumulating
          // across the common chain gives the total world rotation at the
          // query. Bend-rotation's transport-to-translation contribution is
          // already captured by the *Rot columns above.
          const rotLocalX = to.rot[0] + bx.rot[0] + by.rot[0];
          const rotLocalY = to.rot[1] + bx.rot[1] + by.rot[1];
          const rotLocalZ = to.rot[2] + bx.rot[2] + by.rot[2];
          const rW0 = R[0] * rotLocalX + R[1] * rotLocalY + R[2] * rotLocalZ;
          const rW1 = R[3] * rotLocalX + R[4] * rotLocalY + R[5] * rotLocalZ;
          const rW2 = R[6] * rotLocalX + R[7] * rotLocalY + R[8] * rotLocalZ;
          const rotKey = `${q},${p}`;
          let rotT = rotMap.get(rotKey);
          if (!rotT) { rotT = newMat3(); rotMap.set(rotKey, rotT); }
          rotT[0 + j] = (rotT[0 + j] as number) + rW0;
          rotT[3 + j] = (rotT[3 + j] as number) + rW1;
          rotT[6 + j] = (rotT[6 + j] as number) + rW2;
        }

        accumEntry(entryMap, q, p, s.beamIx, 'twist',       eTwist,       totalC);
        accumEntry(entryMap, q, p, s.beamIx, 'bendIxTrans', eBendIxTrans, totalC);
        accumEntry(entryMap, q, p, s.beamIx, 'bendIxRot',   eBendIxRot,   totalC);
        accumEntry(entryMap, q, p, s.beamIx, 'bendIyTrans', eBendIyTrans, totalC);
        accumEntry(entryMap, q, p, s.beamIx, 'bendIyRot',   eBendIyRot,   totalC);
      }

      totalsMap.set(`${q},${p}`, totalC);
    }
  }

  // Materialize entries from the (q, p, b, m) accumulator.
  const entries: ComplianceEntry[] = [];
  for (const [key, C] of entryMap) {
    if (isZeroMat(C)) continue;
    const [qStr, pStr, bStr, mStr] = key.split(',');
    entries.push({
      queryIx: Number(qStr),
      loadIx: Number(pStr),
      beamIx: Number(bStr),
      mode: mStr as Mode,
      C,
    });
  }

  // ---------- Fixed-fixed compatibility solve ----------
  if (fixedFixed) {
    const clampWorld = seg.nodes[clampNodeIx]!.pos_mm;
    const ok = applyFixedFixed(
      beamCount, queryNodeIxs.length, realLoadCount,
      entries, totalsMap, rotMap,
      clampQueryIx, clampForceLoadIx, clampMomentLoadIx, clampWorld,
    );
    if (!ok) {
      return {
        kind: 'error',
        code: 'support-singular',
        message:
          'support(both): the chord is degenerate or the fixed-fixed system is singular ' +
          '— check for a zero-length root beam or a zero-stiffness section',
      };
    }
    // Drop synthetic clamp loads from public API.
    loadFmax_N.length = realLoadCount;
    for (let q = 0; q < queryNodeIxs.length; q++) {
      rotMap.delete(`${q},${clampForceLoadIx}`);
      rotMap.delete(`${q},${clampMomentLoadIx}`);
    }
  }

  const totals = Array.from(totalsMap.entries()).map(([key, C]) => {
    const [q, p] = key.split(',').map(Number) as [number, number];
    return { queryIx: q, loadIx: p, C };
  });
  const rotationTotals = Array.from(rotMap.entries()).map(([key, C]) => {
    const [q, p] = key.split(',').map(Number) as [number, number];
    return { queryIx: q, loadIx: p, C };
  });

  return { loadFmax_N, entries, totals, rotationTotals };
}

function accumEntry(
  map: Map<string, Mat3>,
  q: number, p: number, b: number, mode: Mode,
  C: Mat3, totalC: Mat3,
): void {
  if (isZeroMat(C)) return;
  const key = `${q},${p},${b},${mode}`;
  let agg = map.get(key);
  if (!agg) { agg = newMat3(); map.set(key, agg); }
  addMat(agg, C);
  addMat(totalC, C);
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
  beamCount: number,
  queryCount: number,
  realLoadCount: number,
  entries: ComplianceEntry[],
  totalsMap: Map<string, Mat3>,
  rotMap: Map<string, Mat3>,
  clampQueryIx: number,
  clampForceLoadIx: number,
  clampMomentLoadIx: number,
  clampWorld: Vec3,
): boolean {
  const chordLen = Math.hypot(clampWorld[0], clampWorld[1], clampWorld[2]);
  if (chordLen < 1e-9) return false;
  const e: Vec3 = [clampWorld[0] / chordLen, clampWorld[1] / chordLen, clampWorld[2] / chordLen];

  // Orthonormal basis of the chord-perpendicular plane.
  const [u, v] = orthoBasisFromChord(e);

  const C_FF = totalsMap.get(`${clampQueryIx},${clampForceLoadIx}`);
  const C_FM = totalsMap.get(`${clampQueryIx},${clampMomentLoadIx}`);
  const K_FF = rotMap.get(`${clampQueryIx},${clampForceLoadIx}`);
  const K_FM = rotMap.get(`${clampQueryIx},${clampMomentLoadIx}`);
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
  const R_world: Mat3[] = [];
  const M_world: Mat3[] = [];

  for (let p = 0; p < realLoadCount; p++) {
    const C_tp = totalsMap.get(`${clampQueryIx},${p}`) ?? newMat3();
    const K_tp = rotMap.get(`${clampQueryIx},${p}`) ?? newMat3();
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
  const allModes: Mode[] = ['twist', 'bendIxTrans', 'bendIxRot', 'bendIyTrans', 'bendIyRot'];
  const adjusted = new Map<string, Mat3>();
  for (let q = 0; q < queryCount; q++) {
    for (let p = 0; p < realLoadCount; p++) {
      for (let b = 0; b < beamCount; b++) {
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

  // Adjust rotation totals analogously: rotation at q from real load p =
  // direct + via clamp-force · R + via clamp-moment · M. Rotation isn't
  // decomposed per (b, m) so we work at the (q, p) level directly.
  const rotAdjusted = new Map<string, Mat3>();
  for (let q = 0; q < queryCount; q++) {
    const viaCF = rotMap.get(`${q},${clampForceLoadIx}`);
    const viaCM = rotMap.get(`${q},${clampMomentLoadIx}`);
    for (let p = 0; p < realLoadCount; p++) {
      const direct = rotMap.get(`${q},${p}`);
      if (!direct && !viaCF && !viaCM) continue;
      const out = newMat3();
      if (direct) addMat(out, direct);
      if (viaCF)  addMat(out, matMul3(viaCF, R_world[p]!));
      if (viaCM)  addMat(out, matMul3(viaCM, M_world[p]!));
      if (isZeroMat(out)) continue;
      rotAdjusted.set(`${q},${p}`, out);
    }
  }
  // Preserve clamp-load rotation rows; the outer caller will delete those
  // after the adjusted real-load rows are in place.
  const cfRows: [string, Mat3][] = [];
  const cmRows: [string, Mat3][] = [];
  for (let q = 0; q < queryCount; q++) {
    const cf = rotMap.get(`${q},${clampForceLoadIx}`);
    const cm = rotMap.get(`${q},${clampMomentLoadIx}`);
    if (cf) cfRows.push([`${q},${clampForceLoadIx}`, cf]);
    if (cm) cmRows.push([`${q},${clampMomentLoadIx}`, cm]);
  }
  rotMap.clear();
  for (const [k, C] of rotAdjusted) rotMap.set(k, C);
  for (const [k, C] of cfRows) rotMap.set(k, C);
  for (const [k, C] of cmRows) rotMap.set(k, C);
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
// Cantilever of length L along +Z (fixed at base z=0), tip-applied (F, M) in
// beam-local frame with RHR moments. Returned `defl` and `rot` are at the
// tip (s = L). The segment walker places every load and query at a node, so
// the cantilever's "load" and "eval" both sit at the tip — no interior-s
// formula needed.
//
// Axial extension/compression is not modeled — see README scope notes.
// Chord-aligned forces produce no deflection contribution; the beam is
// treated as axially rigid.

interface ModeOut { defl: Vec3; rot: Vec3 }
const ZERO: ModeOut = { defl: [0, 0, 0], rot: [0, 0, 0] };

function modeTorsion(M: Vec3, L: number, G_MPa: number, J_mm4: number): ModeOut {
  if (J_mm4 <= 0 || G_MPa <= 0) return ZERO;
  // θ_z(L) = M_z · L / (G·J). Pure torsion has no deflection contribution.
  return { defl: [0, 0, 0], rot: [0, 0, M[2] * L / (G_MPa * J_mm4)] };
}

// Shared Euler-Bernoulli cantilever-tip kernel for the two bending modes.
//
// `sigma` ties signs to (axis_perp, axis_about, axial) chirality, governing
// two relations at once:
//   1. rot_about_RHR = sigma * du_perp/ds. For (perp=y, about=x, axial=z) —
//      modeBendIx, sigma=-1 — positive du_perp/ds (beam tilts toward +y)
//      corresponds to *negative* RHR rotation about +x, since +x rotation
//      takes +z toward -y. For (perp=x, about=y, axial=z) — modeBendIy,
//      sigma=+1 — du_perp/ds and rot match in sign.
//   2. The bending moment carries +sigma * M_about: an RHR tip moment M_x
//      (sigma=-1, modeBendIx) drives u_y *negative* (-M·L²/(2EI)); an RHR
//      tip moment M_y (sigma=+1, modeBendIy) drives u_x *positive*
//      (+M·L²/(2EI)). Same sigma value gates both because both flow from
//      the same chirality.
function modeBend(
  F_perp: number, M_about: number,
  L: number, EI: number, sigma: -1 | 1,
): { u_perp: number; rot_about: number } {
  const u_perp = (F_perp * L * L * L / 3 + sigma * M_about * L * L / 2) / EI;
  const du_ds  = (F_perp * L * L     / 2 + sigma * M_about * L)         / EI;
  return { u_perp, rot_about: sigma * du_ds };
}

function modeBendIx(F: Vec3, M: Vec3, L: number, E_MPa: number, Ix_mm4: number): ModeOut {
  if (Ix_mm4 <= 0 || E_MPa <= 0) return ZERO;
  const r = modeBend(F[1], M[0], L, E_MPa * Ix_mm4, -1);
  return { defl: [0, r.u_perp, 0], rot: [r.rot_about, 0, 0] };
}

function modeBendIy(F: Vec3, M: Vec3, L: number, E_MPa: number, Iy_mm4: number): ModeOut {
  if (Iy_mm4 <= 0 || E_MPa <= 0) return ZERO;
  const r = modeBend(F[0], M[1], L, E_MPa * Iy_mm4, +1);
  return { defl: [r.u_perp, 0, 0], rot: [0, r.rot_about, 0] };
}

// ---------- world transport ----------
//
// World deflection at the query from one (beam, mode) = R·defl_local +
// (R·rot_local) × arm. Split into two rank-1-preserving pieces so the
// per-(q, p, i, mode) Mat3 stays rank ≤ 1: translationOnly is along a fixed
// world direction (a column of R); transportOnly is along (R·column × arm).
// The 5-mode breakdown semantics rely on that rank-1 property.

function translationOnly(R: Mat3, defl_local: Vec3): Vec3 {
  return matVec(R, defl_local);
}

function transportOnly(R: Mat3, rot_local: Vec3, arm_world: Vec3): Vec3 {
  const rotW = matVec(R, rot_local);
  return crossV(rotW, arm_world);
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

function subV(a: Vec3, b: Vec3): Vec3 { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
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
