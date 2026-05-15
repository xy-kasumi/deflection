// Anchor tests for sim/segment.ts. Pins node/segment counts, adjacency
// invariants, and the input-validation error codes — internal graph
// structure that properties.test.ts doesn't probe (it only sees
// simulate's public output, not the segmentation graph).
// Run with: npx tsx tests/sim/segment.test.ts

import type { Beam, Problem } from '../../src/sim/problem';
import type { Vec3 } from '../../src/sim/math';
import { buildSegmentation } from '../../src/sim/segment';
import { expect, eqInt, near, finish } from './_assert';

const STEEL = { E_MPa: 200_000, G_MPa: 79_000 };
const SECT = { Ix_mm4: 833.33, Iy_mm4: 833.33, J_mm4: 1400 };

function horzBeam(origin_mm: Vec3, length_mm: number): Beam {
  return {
    frame: { origin_mm, ex: [0, 0, -1], ey: [0, 1, 0], axial: [1, 0, 0] },
    length_mm,
    section: SECT,
    material: STEEL,
  };
}

// ---- 1: empty problem ----
{
  console.log('-- empty --');
  const r = buildSegmentation({ beams: [], loads: [], queries: [], support: 'single' });
  if ('kind' in r) { expect('not an error', false); }
  else {
    eqInt('no nodes', r.nodes.length, 0);
    eqInt('no segments', r.segments.length, 0);
  }
}

// ---- 2: single bare beam ----
{
  console.log('\n-- single bare beam --');
  const r = buildSegmentation({
    beams: [horzBeam([0, 0, 0], 100)],
    loads: [], queries: [], support: 'single',
  });
  if ('kind' in r) { expect('not an error', false); }
  else {
    eqInt('2 nodes (endpoints)', r.nodes.length, 2);
    eqInt('1 segment', r.segments.length, 1);
    eqInt('beamStartNode[0] is root', r.beamStartNode[0]!, 0);
    near('seg length = 100', r.segments[0]!.length_mm, 100);
    eqInt('seg.beamIx = 0', r.segments[0]!.beamIx, 0);
  }
}

// ---- 3: single beam with one mid load + tip query ----
{
  console.log('\n-- 1 beam + 1 load + 1 query --');
  const r = buildSegmentation({
    beams: [horzBeam([0, 0, 0], 100)],
    loads: [{ beamIx: 0, offset_mm: 50, Fmax_N: 1 }],
    queries: [{ beamIx: 0, offset_mm: 100 }],
    support: 'single',
  });
  if ('kind' in r) { expect('not an error', false); }
  else {
    eqInt('nodes: 0, 50, 100', r.nodes.length, 3);
    eqInt('2 segments', r.segments.length, 2);
    near('seg0 length', r.segments[0]!.length_mm, 50);
    near('seg1 length', r.segments[1]!.length_mm, 50);
    eqInt('1 segLoad', r.loads.length, 1);
    eqInt('load loadIx = 0', r.loads[0]!.loadIx, 0);
    eqInt('1 segQuery', r.queries.length, 1);
    expect('load.nodeIx is the mid node', r.nodes[r.loads[0]!.nodeIx]!.pos_mm[0] === 50);
    expect('query.nodeIx is the tip node', r.nodes[r.queries[0]!.nodeIx]!.pos_mm[0] === 100);
  }
}

// ---- 4: tip-to-tip two-beam chain (no splits) ----
{
  console.log('\n-- tip-to-tip 2-beam --');
  const beam1: Beam = {
    frame: { origin_mm: [100, 0, 0], ex: [0, 0, -1], ey: [-1, 0, 0], axial: [0, 1, 0] },
    length_mm: 100, section: SECT, material: STEEL,
  };
  const r = buildSegmentation({
    beams: [horzBeam([0, 0, 0], 100), beam1],
    loads: [], queries: [], support: 'single',
  });
  if ('kind' in r) { expect('not an error', false); }
  else {
    eqInt('3 nodes', r.nodes.length, 3);
    eqInt('2 segments', r.segments.length, 2);
    eqInt('beam1 start = beam0 end (shared node)', r.beamStartNode[1]!, r.beamEndNode[0]!);
  }
}

// ---- 5: mid-attached two-beam chain ----
{
  console.log('\n-- mid-attached 2-beam --');
  // beam1 starts at beam0[100]; beam0 has L=300, so beam0 splits at 100.
  const beam1: Beam = {
    frame: { origin_mm: [100, 0, 0], ex: [1, 0, 0], ey: [0, 1, 0], axial: [0, 0, 1] },
    length_mm: 100, section: SECT, material: STEEL,
  };
  const r = buildSegmentation({
    beams: [horzBeam([0, 0, 0], 300), beam1],
    loads: [{ beamIx: 1, offset_mm: 100, Fmax_N: 1 }],
    queries: [{ beamIx: 1, offset_mm: 0 }],
    support: 'single',
  });
  if ('kind' in r) { expect('not an error', false); }
  else {
    // beam0: nodes at 0, 100, 300 → 2 segments.
    // beam1: nodes at 0, 100 → 1 segment.
    // shared: beam0's mid node = beam1's start node.
    eqInt('nodes total (beam0: 3, beam1: 1 new + shared)', r.nodes.length, 4);
    eqInt('3 segments', r.segments.length, 3);
    eqInt('beam1 start node = beam0 mid (offset 100)', r.beamStartNode[1]!, r.beamEndNode[0]! - 1);
    // The first two segs are beam0's; segment2 is beam1's.
    eqInt('seg0 beamIx', r.segments[0]!.beamIx, 0);
    eqInt('seg1 beamIx (beam0 tail past mid)', r.segments[1]!.beamIx, 0);
    eqInt('seg2 beamIx (beam1)', r.segments[2]!.beamIx, 1);
    near('seg0 length 100', r.segments[0]!.length_mm, 100);
    near('seg1 length 200', r.segments[1]!.length_mm, 200);
    near('seg2 length 100', r.segments[2]!.length_mm, 100);
    expect('load.nodeIx is beam1 tip', r.loads[0]!.nodeIx === r.beamEndNode[1]);
    expect('query.nodeIx is beam1 start (= beam0 mid)', r.queries[0]!.nodeIx === r.beamStartNode[1]);
  }
}

// ---- 6: off-axis child → SimError ----
{
  console.log('\n-- disconnected (axis-off) → error --');
  const r = buildSegmentation({
    beams: [horzBeam([0, 0, 0], 100), horzBeam([0, 50, 0], 100)],
    loads: [], queries: [], support: 'single',
  });
  expect('returns error', 'kind' in r && r.kind === 'error' && r.code === 'beams-disconnected');
}

finish('segment anchors');
