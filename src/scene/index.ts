import * as THREE from 'three';
import type { BeamNode, Vec3 } from '../walker';
import type { SimResult } from '../sim/simulate';
import { COLOR, VU, MOTION } from './tokens';
import { Labels } from './labels';
import { LobeRenderer, computeLobeCeilWorld } from './lobe';
import { StickRenderer, type Stick } from './stick';
import { buildChain } from './chain';

const ISO_YAW_DEG = 45;
const ISO_PITCH_DEG = -30;

// Click vs drag: pointerup with movement below this threshold (squared, px) is a click.
const CLICK_MOVE_THRESH_SQ = 16;
const YAW_PER_PX = 1 / 150;

const deg = (d: number) => (d * Math.PI) / 180;

export class Scene {
  private renderer: THREE.WebGLRenderer;
  private root: THREE.Scene;
  private camera: THREE.OrthographicCamera;
  private content: THREE.Group;
  private yaw: number;
  private pitch: number;
  private targetYaw: number;
  private yawVelocity = 0;
  private dragging = false;
  private lastPointerX = 0;
  private lastFrameTime = 0;
  private animHandle = 0;
  private center: THREE.Vector3 = new THREE.Vector3(0, 0, 0);
  private scaleHalf = 100;

  private onPick: (nodeIx: number) => void;
  private pickables: THREE.Mesh[] = [];
  private raycaster = new THREE.Raycaster();
  private labels: Labels;
  private lobes = new LobeRenderer();
  private sticks = new StickRenderer();

  constructor(canvas: HTMLCanvasElement, onPick: (nodeIx: number) => void) {
    this.onPick = onPick;

    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.root = new THREE.Scene();
    this.root.background = new THREE.Color(COLOR.bg);
    this.camera = new THREE.OrthographicCamera(-1, 1, 1, -1, -10000, 10000);
    this.content = new THREE.Group();
    this.root.add(this.content);
    // Stick overlay sits at the root, *outside* content. content is disposed
    // and rebuilt on every update(); the stick group survives so cursor-driven
    // sticks don't blink when an unrelated rebuild happens.
    this.root.add(this.sticks.getGroup());
    this.yaw = deg(ISO_YAW_DEG);
    this.pitch = deg(ISO_PITCH_DEG);
    this.targetYaw = this.yaw;

    // Take focus on canvas interaction so CodeMirror's `focusChanged` event
    // fires — that's what flips the scene into inspection mode.
    canvas.tabIndex = -1;
    canvas.style.outline = 'none';

    this.labels = new Labels(canvas.parentElement!);

    const ro = new ResizeObserver(() => this.onResize());
    ro.observe(canvas);

    this.installDrag(canvas);
  }

  update(
    beams: BeamNode[],
    sim: SimResult | undefined,
    supportKind: 'single' | 'both' | undefined,
    editor: { currentBeamIx: number | null; focused: boolean } | undefined,
    selectedNodeIx: number,
  ): void {
    disposeChildren(this.content);
    this.pickables = [];
    this.lobes.reset();
    this.labels.clear();

    if (beams.length === 0) {
      this.center.set(0, 0, 0);
      this.scaleHalf = 100;
      this.refresh();
      return;
    }

    const focused = editor?.focused === true;

    // Visual unit u = rod diameter; everything visible is expressed as u × k
    // via the VU tokens. `avgL/40` is the only place avgL touches geometry;
    // the floor keeps glyphs visible on degenerately short chains.
    const avgL = beams.reduce((s, b) => s + b.length_mm, 0) / beams.length;
    const u = Math.max(1, avgL / 40);

    const hitRadius   = u * VU.hitR;
    const labelOffset = u * VU.labelOffset;
    const lobeFloor   = u * VU.lobeFloorR;
    this.sticks.setRadius(u * VU.stickR);

    const chain = buildChain(beams, {
      supportKind,
      focused,
      currentBeamIx: editor?.currentBeamIx ?? null,
      u,
    });
    for (const m of chain.meshes) this.content.add(m);

    chain.bbox.getCenter(this.center);
    const size = chain.bbox.getSize(new THREE.Vector3());
    const maxDim = Math.max(size.x, size.y, size.z);
    this.scaleHalf = Math.max(50, maxDim * 0.8 + Math.max(u * 6, 10));

    if (sim && sim.queryResults.length > 0) {
      const r = this.lobes.buildFor(sim, beams, {
        hitRadius, labelOffset, lobeFloor, selectedNodeIx, focused,
      });
      for (const m of r.meshes) this.content.add(m);
      this.pickables.push(...r.pickables);
      for (const l of r.labels) {
        const pickIx = l.nodeIx;
        this.labels.add(l.text, l.worldPos, {
          classes: l.classes,
          onClick: () => this.onPick(pickIx),
        });
      }
    }

    // apply() writes scale/opacity onto the freshly built meshes; it must run
    // before refresh() or the first rendered frame shows lobes at their
    // default mesh.scale = 1 (konpeito at 1mm, etc.). Canvas-side pointerdown
    // kicks the rAF loop and hides this; label-click clearly exposes it
    // because nothing else triggers a re-render.
    this.applyLobesAndSticks();
    this.refresh();
  }

  setDisplayScale(target: number): void {
    if (this.lobes.setTarget(target)) this.startAnim();
  }

  // Show / hide contribution sticks. Pessimistic decomposition has independent
  // (query, beam, mode) rank-1 contributions, so a single cell hover may
  // emit one stick per query × one mode = N_queries sticks; a per-beam total
  // hover emits N_queries × 5 sticks. apply() runs once so the fresh meshes
  // pick up the live δ-exag immediately.
  setSticks(sticks: Stick[] | null): void {
    const kick = this.sticks.setSticks(sticks);
    // Floating δ-labels (DOM, in front of canvas) would otherwise cover the
    // contribution stick; cross-fade them out while it's on screen. CSS owns
    // the label transition, so toggling the class once is enough.
    this.labels.setFaded(!!sticks && sticks.length > 0);
    // apply() is mandatory after setSticks whenever geometry was rebuilt —
    // fresh cylinders ship at mesh.scale = (1,1,1) (length = 1 mm), so without
    // it cell-to-cell hovers flash a too-small stick until something else ticks.
    this.applyLobesAndSticks();
    if (kick) this.startAnim();
    else this.refresh();
  }

  recommendDisplayScale(delta_max_mm: number): number {
    const ceil = computeLobeCeilWorld(this.renderer.domElement, this.scaleHalf);
    return this.lobes.recommendDisplayScale(delta_max_mm, ceil);
  }

  private tryPick(clientX: number, clientY: number): void {
    if (this.pickables.length === 0) return;
    const rect = this.renderer.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(
       ((clientX - rect.left) / rect.width)  * 2 - 1,
      -((clientY - rect.top)  / rect.height) * 2 + 1,
    );
    this.content.updateMatrixWorld(true);
    this.raycaster.setFromCamera(ndc, this.camera);
    const hits = this.raycaster.intersectObjects(this.pickables, false);
    if (hits.length > 0) {
      const nodeIx = hits[0]!.object.userData['nodeIx'] as number;
      this.onPick(nodeIx);
    }
  }

  private installDrag(canvas: HTMLCanvasElement) {
    let downX = 0;
    let downY = 0;

    canvas.addEventListener('pointerdown', (e) => {
      downX = e.clientX;
      downY = e.clientY;
      canvas.focus();
      this.dragging = true;
      this.lastPointerX = e.clientX;
      this.yawVelocity = 0;
      this.targetYaw = this.yaw;
      canvas.setPointerCapture(e.pointerId);
      canvas.classList.add('dragging');
      this.startAnim();
    });

    canvas.addEventListener('pointermove', (e) => {
      if (!this.dragging) return;
      const dx = e.clientX - this.lastPointerX;
      this.lastPointerX = e.clientX;
      this.targetYaw -= dx * YAW_PER_PX;
    });

    const end = (e: PointerEvent) => {
      if (!this.dragging) return;
      this.dragging = false;
      canvas.classList.remove('dragging');
      const dx = e.clientX - downX;
      const dy = e.clientY - downY;
      if (dx * dx + dy * dy < CLICK_MOVE_THRESH_SQ) {
        this.tryPick(e.clientX, e.clientY);
      }
    };
    canvas.addEventListener('pointerup', end);
    canvas.addEventListener('pointercancel', end);
  }

  private startAnim() {
    if (this.animHandle) return;
    this.lastFrameTime = performance.now();
    const tick = (now: number) => {
      const dt = Math.min(0.05, (now - this.lastFrameTime) / 1000);
      this.lastFrameTime = now;

      if (this.dragging) {
        const prevYaw = this.yaw;
        const alpha = 1 - Math.exp(-dt * MOTION.dragSmoothK);
        this.yaw += (this.targetYaw - this.yaw) * alpha;
        this.yawVelocity = (this.yaw - prevYaw) / Math.max(dt, 1e-3);
      } else {
        this.yaw += this.yawVelocity * dt;
        this.targetYaw = this.yaw;
        this.yawVelocity *= Math.exp(-dt * MOTION.dragDecayK);
        if (Math.abs(this.yawVelocity) < MOTION.dragStopVel) this.yawVelocity = 0;
      }

      const scaleSettled = this.lobes.tickScale(dt);
      const stickSettled = this.sticks.tickFade(dt);
      this.applyLobesAndSticks();

      this.refresh();

      const settled = !this.dragging
        && this.yawVelocity === 0
        && Math.abs(this.targetYaw - this.yaw) < 1e-4
        && scaleSettled
        && stickSettled;
      if (settled) {
        this.animHandle = 0;
        return;
      }
      this.animHandle = requestAnimationFrame(tick);
    };
    this.animHandle = requestAnimationFrame(tick);
  }

  // Lobes and sticks share the canvas-derived ceiling and the animated δ-exag,
  // and always need to be applied together (sticks read scale via the lobe's
  // getDisplayScale()). Wrapping keeps every call site in lockstep.
  private applyLobesAndSticks() {
    const ceil = computeLobeCeilWorld(this.renderer.domElement, this.scaleHalf);
    this.lobes.apply(ceil);
    this.sticks.apply(this.lobes.getDisplayScale(), ceil);
  }

  private refresh() {
    this.updateCamera();
    this.renderer.render(this.root, this.camera);
    this.labels.update(this.camera, this.renderer.domElement);
  }

  private updateCamera() {
    const target = this.center;
    const elev = -this.pitch;
    const ce = Math.cos(elev);
    const se = Math.sin(elev);
    const sy = Math.sin(this.yaw);
    const cy = Math.cos(this.yaw);
    const r = 4 * this.scaleHalf;
    this.camera.position.set(
      target.x + r * ce * sy,
      target.y + r * se,
      target.z + r * ce * cy,
    );
    if (Math.abs(ce) < 0.01) {
      this.camera.up.set(sy, 0, cy);
    } else {
      this.camera.up.set(0, 1, 0);
    }
    this.camera.lookAt(target);

    const canvas = this.renderer.domElement;
    const w = canvas.clientWidth || 1;
    const h = canvas.clientHeight || 1;
    const aspect = w / h;
    const halfBase = this.scaleHalf;
    if (aspect >= 1) {
      this.camera.left = -halfBase * aspect;
      this.camera.right = halfBase * aspect;
      this.camera.top = halfBase;
      this.camera.bottom = -halfBase;
    } else {
      this.camera.left = -halfBase;
      this.camera.right = halfBase;
      this.camera.top = halfBase / aspect;
      this.camera.bottom = -halfBase / aspect;
    }
    this.camera.near = -100 * this.scaleHalf;
    this.camera.far = 100 * this.scaleHalf;
    this.camera.updateProjectionMatrix();
  }

  private onResize() {
    const canvas = this.renderer.domElement;
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (w > 0 && h > 0) {
      this.renderer.setSize(w, h, false);
    }
    this.refresh();
  }
}

function disposeChildren(group: THREE.Group) {
  group.traverse((obj) => {
    const o = obj as THREE.Object3D & {
      geometry?: THREE.BufferGeometry;
    };
    if (o.geometry) o.geometry.dispose();
    // Materials are intentionally NOT disposed here. THREE's WebGLPrograms
    // cache is refcounted: disposing every material of a given kind drops the
    // program refcount to zero, deleting the program, forcing a recompile on
    // the next build (~30ms across our basic-material zoo). Letting materials
    // become unreferenced and GC'd preserves the program cache. Scene
    // materials are small (no textures, simple uniforms) and the build
    // frequency is human-driven, so memory growth is negligible.
  });
  group.clear();
}

// Keep this export so callers using ...spread-style construction can pass a Vec3.
export type { Vec3 };
