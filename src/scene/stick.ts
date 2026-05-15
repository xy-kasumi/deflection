import * as THREE from 'three';
import type { Vec3 } from '../walker';
import { COLOR, MOTION, easeToward } from './tokens';

// One straight segment anchored at a query node, oriented along the unit
// argmax direction d* of a single (beam, mode) part, half-length δ_{b,m} (mm)
// at unit δ-exag. The rank-1 contribution |M·d| is symmetric in d, so the
// renderer draws ±vector_mm from origin — the segment crosses the node and
// pokes out the antipode of the lobe. The renderer scales it with the live
// δ-exag and clamps each tip at lobeCeilWorld so it never shoots off-screen.
export interface Stick {
  origin_mm: Vec3;
  /** d* · δ_{b,m} — half-vector; the segment extends ±this from origin at unit δ-exag. */
  vector_mm: Vec3;
}

// Renders the contribution sticks: a fade-in/out overlay that lives in a
// group the Scene parents directly to its root (outside `content`) so it
// survives chain/lobe rebuilds. The material is reused across changes to keep
// THREE's program cache warm. Each child cylinder carries its unscaled mm
// half-length in `userData.dist_mm` so apply() can clamp it individually
// against lobeCeilWorld; the cross-section is fixed in world units (set via
// setRadius from chain's `u`) so δ-exag scales length without fattening.
//
// fade_target/anim drives an opacity transition on `mat`: setSticks flips
// the target, the anim loop eases fade_anim toward it, and the children are
// disposed only after the fade-out has fully completed (so the sticks don't
// snap out before the user's eye has tracked them away).
export class StickRenderer {
  private group = new THREE.Group();
  private mat = new THREE.MeshBasicMaterial({
    color: COLOR.deformed,
    transparent: true,
    opacity: 0,
  });
  private radius = 0;
  private fade_target = 0;
  private fade_anim = 0;

  getGroup(): THREE.Group {
    return this.group;
  }

  setRadius(r: number): void {
    this.radius = r;
  }

  // Set the sticks to display. Geometry is rebuilt eagerly when sticks is
  // non-empty so the next tick already has the right meshes to fade in; on
  // null/empty, the existing children are kept until the fade-out finishes,
  // so the user sees them ease away instead of snap. Returns true if the
  // animation loop should be kicked.
  setSticks(sticks: Stick[] | null): boolean {
    if (sticks && sticks.length > 0) {
      this.rebuildChildren(sticks);
      const changed = this.fade_target !== 1 || this.fade_anim !== 1;
      this.fade_target = 1;
      return changed;
    }
    const changed = this.fade_target !== 0 || this.fade_anim !== 0;
    this.fade_target = 0;
    return changed;
  }

  // Cylinder per Stick, oriented local-Y = d* so apply() can scale only
  // mesh.scale.y without inflating the cross-section. Clamp uses |v| (one-side
  // length); both tips stay inside the konpeito ceiling.
  private rebuildChildren(sticks: Stick[]): void {
    this.disposeChildren();
    const r = this.radius;
    const yAxis = new THREE.Vector3(0, 1, 0);
    for (const s of sticks) {
      const v = s.vector_mm;
      const dist = Math.hypot(v[0], v[1], v[2]);
      if (dist === 0) continue;
      // Unit-length cylinder along local Y; apply() scales Y to 2·dist·δ-exag.
      const geom = new THREE.CylinderGeometry(r, r, 1, 12);
      const mesh = new THREE.Mesh(geom, this.mat);
      mesh.quaternion.setFromUnitVectors(
        yAxis,
        new THREE.Vector3(v[0] / dist, v[1] / dist, v[2] / dist),
      );
      mesh.position.set(s.origin_mm[0], s.origin_mm[1], s.origin_mm[2]);
      mesh.userData['dist_mm'] = dist;
      this.group.add(mesh);
    }
  }

  private disposeChildren(): void {
    for (const child of this.group.children) {
      const o = child as THREE.Object3D & { geometry?: THREE.BufferGeometry };
      o.geometry?.dispose();
    }
    this.group.clear();
  }

  // Ease fade_anim toward target; dispose children once a fade-out has
  // fully settled. Returns true when the fade is settled (no further frames
  // needed). Counterpart to LobeRenderer.tickScale for the anim loop's
  // settled check.
  tickFade(dt: number): boolean {
    const tgt = this.fade_target;
    const cur = this.fade_anim;
    if (cur === tgt) return true;
    let next = easeToward(cur, tgt, dt, MOTION.stickFadeK);
    if (Math.abs(next - tgt) < MOTION.stickFadeSettle) next = tgt;
    this.fade_anim = next;
    if (next === 0 && tgt === 0) this.disposeChildren();
    return next === tgt;
  }

  // Per-frame: opacity from fade, length from δ-exag scale, each clamped at
  // lobeCeilWorld so a big mode at a high exag setting can't shoot past the
  // konpeito. Clamp is per-tip (dist is half-length), and only the length
  // axis scales — the cross-section radius is baked into the geometry in
  // world units.
  apply(scale: number, lobeCeilWorld: number): void {
    this.mat.opacity = this.fade_anim;
    for (const child of this.group.children) {
      const dist = child.userData['dist_mm'] as number | undefined;
      if (!dist || dist <= 0) continue;
      const s = Math.min(scale, lobeCeilWorld / dist);
      child.scale.set(1, 2 * dist * s, 1);
    }
  }
}
