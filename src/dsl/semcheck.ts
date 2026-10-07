import type { Diagnostic, Span } from './diagnostics';
import type { Attachment, BeamDef, Param, Structure } from './parse';

const KNOWN_ENVS = new Set(['support', 'mass_accel']);
const ACCEL_UNITS = new Set(['G', 'm/s2']);
const KNOWN_MATERIALS = new Set(['plastic', 'aluminum', 'steel']);
const KNOWN_SHAPES = new Set(['rect', 'round', 'section']);
const KNOWN_ATTACHMENTS = new Set(['load', 'torque']);
const FORCE_AXES = new Set(['x', 'y', 'z', 'xy', 'xz', 'yz', 'xyz']);
const FORCE_UNITS = new Set(['N', 'kgf', 'gf']);
const TORQUE_UNITS = new Set(['Nmm', 'Ncm', 'Nm']);

/**
 * All semantic issues are warnings — the structure still walks/renders, just
 * with sensible fallbacks. Hard syntax errors are emitted at parse time.
 */
export function semcheck(s: Structure): Diagnostic[] {
  const diags: Diagnostic[] = [];

  for (const env of s.envs) {
    if (!KNOWN_ENVS.has(env.name)) {
      diags.push({
        severity: 'warning',
        message: `unknown environment '${env.name}'`,
        span: env.span,
      });
      continue;
    }
    if (env.name === 'mass_accel') checkMassAccel(env.params, env.span, diags);
  }

  for (let i = 0; i < s.beams.length; i++) {
    const b = s.beams[i] as BeamDef;
    const isRoot = i === 0;

    if (isRoot && b.loc) {
      diags.push({
        severity: 'warning',
        message: 'loc-spec ignored on root beam',
        span: b.loc.span,
      });
    }
    if (!isRoot && b.dir === 'horz') {
      diags.push({
        severity: 'warning',
        message: "'horz' is ambiguous on a non-root beam",
        span: b.span,
      });
    }

    checkBeamParams(b, diags);

    for (const a of b.attachments) {
      checkAttachment(a, diags);
    }
  }

  return diags;
}

function checkBeamParams(b: BeamDef, diags: Diagnostic[]) {
  let material: Param | undefined;
  let shape: Param | undefined;
  let length: Param | undefined;

  for (const p of b.params) {
    if (p.kind === 'ident') {
      if (p.params === undefined) {
        // bare ident — material slot
        if (KNOWN_MATERIALS.has(p.name)) {
          if (material) {
            diags.push({
              severity: 'warning',
              message: 'multiple materials in beam args',
              span: p.span,
            });
          } else {
            material = p;
          }
        } else {
          diags.push({
            severity: 'warning',
            message: `unknown identifier '${p.name}'`,
            span: p.span,
          });
        }
      } else {
        // function-call form — shape slot
        if (KNOWN_SHAPES.has(p.name)) {
          if (shape) {
            diags.push({
              severity: 'warning',
              message: 'multiple shapes in beam args',
              span: p.span,
            });
          } else {
            shape = p;
            checkShapeArgs(p.name, p.params, diags);
          }
        } else {
          diags.push({
            severity: 'warning',
            message: `unknown shape function '${p.name}'`,
            span: p.span,
          });
        }
      }
    } else {
      // quantity — length slot (prefix L)
      const q = p.quantity;
      if (q.prefix === 'L') {
        if (length) {
          diags.push({
            severity: 'warning',
            message: "multiple lengths (prefix 'L')",
            span: p.span,
          });
        } else {
          length = p;
        }
      } else {
        diags.push({
          severity: 'warning',
          message: q.prefix
            ? `prefix '${q.prefix}' not expected in beam args`
            : "bare number in beam args (use 'L' for length)",
          span: p.span,
        });
      }
    }
  }

  if (!material) {
    diags.push({ severity: 'warning', message: 'missing material', span: b.span });
  }
  if (!shape) {
    diags.push({ severity: 'warning', message: 'missing shape', span: b.span });
  }
  if (!length) {
    diags.push({
      severity: 'warning',
      message: "missing length (e.g. 'L100') — defaulting to L=100",
      span: b.span,
    });
  }
}

function checkShapeArgs(name: string, params: Param[] | undefined, diags: Diagnostic[]) {
  if (!params) return;
  // Expected prefixes by shape. Each shape has a set of allowed prefixes.
  const allowed: Record<string, Set<string>> = {
    rect: new Set(['W', 'H', 'T']),
    round: new Set(['D', 'T']),
    section: new Set(['I', 'Ix', 'Iy', 'J', 'A']),
  };
  const set = allowed[name];
  if (!set) return;
  for (const p of params) {
    if (p.kind !== 'quantity') {
      diags.push({
        severity: 'warning',
        message: `expected number with prefix in ${name}(...)`,
        span: p.span,
      });
      continue;
    }
    if (!p.quantity.prefix) {
      diags.push({
        severity: 'warning',
        message: `bare number in ${name}(...) — expected prefix (${[...set].join(', ')})`,
        span: p.span,
      });
      continue;
    }
    if (!set.has(p.quantity.prefix)) {
      diags.push({
        severity: 'warning',
        message: `unexpected prefix '${p.quantity.prefix}' in ${name}(...) — allowed: ${[...set].join(', ')}`,
        span: p.span,
      });
    }
  }
}

function checkMassAccel(params: Param[], span: Span, diags: Diagnostic[]) {
  if (params.length !== 1) {
    diags.push({
      severity: 'warning',
      message: 'mass_accel(...) takes exactly one acceleration',
      span,
    });
    return;
  }
  const p = params[0] as Param;
  if (p.kind !== 'quantity') {
    diags.push({
      severity: 'warning',
      message: 'mass_accel(...) argument must be a number',
      span: p.span,
    });
    return;
  }
  const q = p.quantity;
  if (q.prefix) {
    diags.push({
      severity: 'warning',
      message: `unexpected prefix '${q.prefix}' on mass_accel argument`,
      span: p.span,
    });
  }
  if (q.unit !== undefined && !ACCEL_UNITS.has(q.unit)) {
    diags.push({
      severity: 'warning',
      message: `unknown unit '${q.unit}' on mass_accel (expected G or m/s2)`,
      span: p.span,
    });
  }
  if (!Number.isFinite(q.value) || q.value < 0) {
    diags.push({
      severity: 'warning',
      message: 'mass_accel must be non-negative',
      span: p.span,
    });
  }
}

function checkAttachment(a: Attachment, diags: Diagnostic[]) {
  if (!KNOWN_ATTACHMENTS.has(a.name)) {
    diags.push({
      severity: 'warning',
      message: `unknown attachment '${a.name}'`,
      span: a.span,
    });
    return;
  }
  if (a.name === 'load') {
    if (a.params.length < 1 || a.params.length > 2) {
      diags.push({
        severity: 'warning',
        message: 'load(...) takes a magnitude and optional axes',
        span: a.span,
      });
      return;
    }
    checkMagnitude(a.params[0]!, 'load', FORCE_UNITS, 'kgf, gf, or N', diags);
    const axes = a.params[1];
    if (
      axes &&
      (axes.kind !== 'ident' || axes.params !== undefined || !FORCE_AXES.has(axes.name))
    ) {
      diags.push({
        severity: 'warning',
        message: 'load axes must be one of x, y, z, xy, xz, yz, or xyz',
        span: axes.span,
      });
    }
  } else if (a.name === 'torque') {
    if (a.params.length !== 1) {
      diags.push({
        severity: 'warning',
        message: 'torque(...) takes exactly one magnitude',
        span: a.span,
      });
      return;
    }
    checkMagnitude(a.params[0]!, 'torque', TORQUE_UNITS, 'Nmm, Ncm, or Nm', diags);
  }
}

function checkMagnitude(
  p: Param,
  action: 'load' | 'torque',
  units: Set<string>,
  expectedUnits: string,
  diags: Diagnostic[],
): void {
  if (p.kind !== 'quantity') {
    diags.push({
      severity: 'warning',
      message: `${action}(...) magnitude must be a number`,
      span: p.span,
    });
    return;
  }
  if (p.quantity.prefix) {
    diags.push({
      severity: 'warning',
      message: `unexpected prefix '${p.quantity.prefix}' on ${action} magnitude`,
      span: p.span,
    });
  }
  if (p.quantity.unit && !units.has(p.quantity.unit)) {
    diags.push({
      severity: 'warning',
      message: `unknown unit '${p.quantity.unit}' on ${action} (expected ${expectedUnits})`,
      span: p.span,
    });
  }
  if (!Number.isFinite(p.quantity.value) || p.quantity.value < 0) {
    diags.push({
      severity: 'warning',
      message: `${action} magnitude must be non-negative`,
      span: p.span,
    });
  }
}
