// DSL-to-sim seam anchors for force-axis and torque attachments.
// Run with: npx tsx tests/dsl-actions.test.ts

import { parse } from '../src/dsl/parse';
import { semcheck } from '../src/dsl/semcheck';
import { walk } from '../src/walker';
import { buildLoadSystem } from '../src/loadsystem';
import { expect, eqInt, near, finish } from './sim/_assert';

const src = `mass_accel(0G)
horz beam(steel rect(W10 H10) L100) 25:load(10N xy) mid:load(2N) end:torque(2Nm)
`;
const parsed = parse(src);
const semantic = semcheck(parsed.structure);
const walked = walk(parsed.structure);
const ls = buildLoadSystem(parsed.structure, walked.beams);

expect('action syntax parses', parsed.diagnostics.length === 0);
expect('action syntax passes semantic checks', semantic.length === 0);
eqInt('three actions built', ls.problem.loads.length, 3);

const xy = ls.problem.loads[0]!;
expect(
  'load axes reach sim input',
  xy.kind === 'force' && xy.axes === 'xy' && xy.Fmax_N === 10,
);
near('numeric attachment offset preserved', xy.offset_mm, 25);

const defaultAxes = ls.problem.loads[1]!;
expect(
  'load magnitude alone defaults to xyz',
  defaultAxes.kind === 'force' && defaultAxes.axes === 'xyz' && defaultAxes.Fmax_N === 2,
);

const torque = ls.problem.loads[2]!;
expect(
  'torque converts Nm to Nmm',
  torque.kind === 'torque' && torque.Tmax_Nmm === 2000,
);

const invalid = parse('horz beam(steel rect(W10 H10) L100) load(10N xx)');
const invalidDiags = semcheck(invalid.structure);
expect('invalid axes are diagnosed', invalidDiags.some((d) => d.message.includes('load axes')));

finish('DSL actions');
