// Design tokens — single source of truth for the scene's visual language.
// Algorithmic / mesh-impl knobs live with the concern that owns them; this
// file is only the things a "tweak the look" PR would touch.

// Emotional register of color:
//   neutral / structural        → greys      (bg, joint, clamp, beam)
//   load / physical action      → orange     (attachment, deformed)
//   peak / overflow / off-scale → red        (deformedPeak — *never* "error")
//   editor focus / current      → green/blue (walkerHintCurrent, beamCurrent)
export const COLOR = {
  bg: 0xffffff,
  joint: 0x444b53,
  clamp: 0x444b53,
  beam: 0x9aa0a6,
  beamCurrent: 0x2563eb,
  attachment: 0xd97a1a,
  walkerHintCurrent: 0x2e7d32,
  deformed: 0xd97a1a,
  deformedPeak: 0xc62828,
} as const;

// Multipliers of the visual unit u (= rod diameter, computed per chain from
// avgL). Each value is a radius (geometry constructors take radii); the
// numerical entry equals half the diameter ratio. The comments below state
// the directly-visible diameter so the design language stays readable.
export const VU = {
  rodR: 0.5,           // rod dia = 1u
  jointR: 1.0,         // joint dia = 2u — twice as fat as the rod
  attachR: 0.75,       // attach dia = 1.5u
  clampHalf: 1.1,      // clamp side = 2.2u — just wider than the joint
  hitR: 3,             // hit dia = 6u — generous, also reaches the label
  walkerHintOffset: 5, // along walker-up
  labelOffset: 5,
  lobeFloorR: 0.75,    // underflow dia = 1.5u — just edges past the rod
  stickR: 0.4,         // contribution-stick dia = 0.8u — chunky enough to read past lobe shell and beam
} as const;

// Easing time-constants in the 1/τ form consumed by `easeToward` below.
// dragSmoothK / dragDecayK govern the yaw drag; scaleK governs the log-space
// δ-exag ease. dragStopVel snaps tiny drift to zero so the anim loop can park.
// scaleSettleLog is the tolerance for "settled" in log-units.
export const MOTION = {
  dragSmoothK: 18,
  dragDecayK: 16,
  dragStopVel: 0.1,
  scaleK: 18,
  scaleSettleLog: 1e-3,
  // Stick overlay (contribution stick + matching label fade): τ ≈ 60ms;
  // visible portion of the fade is ~180ms — fast enough to feel responsive,
  // slow enough that the appear/disappear reads as a transition, not a snap.
  stickFadeK: 16,
  stickFadeSettle: 1e-3,
} as const;

// Frame-rate-independent ease toward target. At dt = 1/k, ~63% of the gap
// closes; tune via MOTION.*K. Pass target = 0 for pure decay.
export function easeToward(current: number, target: number, dt: number, k: number): number {
  return current + (target - current) * (1 - Math.exp(-dt * k));
}

// Convert a 0xRRGGBB color token to GLSL `vec3(r, g, b)` literal text.
// Lets the lobe shader paint from COLOR instead of duplicating hex values.
export function hexToVec3(hex: number): string {
  const r = ((hex >> 16) & 0xff) / 255;
  const g = ((hex >> 8) & 0xff) / 255;
  const b = (hex & 0xff) / 255;
  return `vec3(${r.toFixed(4)}, ${g.toFixed(4)}, ${b.toFixed(4)})`;
}
