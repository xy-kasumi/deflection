import type { Diagnostic } from './dsl/diagnostics';
import type { Dir } from './dsl/lex';
import type { Attachment, BeamDef, LocSpec, Structure } from './dsl/parse';

// Walker forward kinematics over the serial beam chain.
//
// World is right-handed with Y up (matches three.js default).
// Frame convention per beam: the ordered triad (F, L, U) is right-handed,
// F × L = U.
// Root is gravity-anchored:
//   horz → F=+X, L=-Z, U=+Y
//   up   → F=+Y, L=-Z, U=-X   (pitch-up from horz)
//   down → F=-Y, L=-Z, U=+X   (pitch-down from horz)
// Non-root: take parent's frame at the attachment offset, then turn:
//   right → F=-L, L=F, U=U     (yaw — walker turns to their right)
//   left  → F=L, L=-F, U=U
//   up    → F=U, U=-F, L=L     (pitch)
//   down  → F=-U, U=F, L=L
//   horz  → no-op (warned upstream)

export type Vec3 = [number, number, number];

export interface Frame {
  origin: Vec3;
  fwd: Vec3;
  left: Vec3;
  up: Vec3;
}

export interface BeamNode {
  def: BeamDef;
  startFrame: Frame;
  length_mm: number;
  material?: string;
  attachmentOffsets: { local_mm: number; def: Attachment }[];
}

export interface WalkResult {
  beams: BeamNode[];
  diagnostics: Diagnostic[];
}

const DEFAULT_L_MM = 100;

const addV = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scaleV = (a: Vec3, s: number): Vec3 => [a[0] * s, a[1] * s, a[2] * s];
const negV = (a: Vec3): Vec3 => [-a[0], -a[1], -a[2]];

export function walk(s: Structure): WalkResult {
  const beams: BeamNode[] = [];
  const diagnostics: Diagnostic[] = [];

  for (let i = 0; i < s.beams.length; i++) {
    const def = s.beams[i] as BeamDef;
    const isRoot = i === 0;

    const { length_mm, material } = extractBeamParams(def);

    let startFrame: Frame;
    if (isRoot) {
      startFrame = rootFrame(def.dir);
    } else {
      const parent = beams[i - 1] as BeamNode;
      const attachOffset = resolveLocOffset(def.loc, parent.length_mm);
      const baseFrame = advanceAlongFwd(parent.startFrame, attachOffset);
      startFrame = applyTurn(baseFrame, def.dir);
    }

    const attachmentOffsets = def.attachments.map((a) => ({
      local_mm: resolveLocOffset(a.loc, length_mm),
      def: a,
    }));

    beams.push({ def, startFrame, length_mm, material, attachmentOffsets });
  }

  return { beams, diagnostics };
}

function rootFrame(dir: Dir): Frame {
  const origin: Vec3 = [0, 0, 0];
  switch (dir) {
    case 'up':
      return { origin, fwd: [0, 1, 0], left: [0, 0, -1], up: [-1, 0, 0] };
    case 'down':
      return { origin, fwd: [0, -1, 0], left: [0, 0, -1], up: [1, 0, 0] };
    default: // horz, and left/right on root fall through to horz
      return { origin, fwd: [1, 0, 0], left: [0, 0, -1], up: [0, 1, 0] };
  }
}

function advanceAlongFwd(f: Frame, mm: number): Frame {
  return {
    origin: addV(f.origin, scaleV(f.fwd, mm)),
    fwd: f.fwd,
    left: f.left,
    up: f.up,
  };
}

function applyTurn(base: Frame, dir: Dir): Frame {
  const { origin, fwd: F, left: L, up: U } = base;
  switch (dir) {
    case 'right':
      return { origin, fwd: negV(L), left: F, up: U };
    case 'left':
      return { origin, fwd: L, left: negV(F), up: U };
    case 'up':
      return { origin, fwd: U, left: L, up: negV(F) };
    case 'down':
      return { origin, fwd: negV(U), left: L, up: F };
    case 'horz':
      // ambiguous on non-root — fall through with no turn so the user can see
      // the rest of the chain (semcheck has already flagged it).
      return base;
    default:
      return base;
  }
}

function resolveLocOffset(loc: LocSpec | undefined, length_mm: number): number {
  if (!loc) return length_mm; // default 'end'
  if (loc.kind === 'kw') {
    return loc.name === 'end' ? length_mm : length_mm / 2;
  }
  return loc.quantity.value;
}

function extractBeamParams(def: BeamDef): { length_mm: number; material?: string } {
  let length_mm = DEFAULT_L_MM;
  let material: string | undefined;
  for (const p of def.params) {
    if (p.kind === 'quantity' && p.quantity.prefix === 'L') {
      length_mm = p.quantity.value;
    } else if (p.kind === 'ident' && p.params === undefined) {
      // Take the first bare ident as material; semcheck flags duplicates.
      if (material === undefined) material = p.name;
    }
  }
  return { length_mm, material };
}
