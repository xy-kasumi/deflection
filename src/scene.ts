import * as THREE from 'three';
import type { BeamState } from './state';
import type { Deflection, CrossSection } from './physics';

const ISO_YAW_DEG = 45;
const ISO_PITCH_DEG = -30;

const COLOR_BG = 0xffffff;
const COLOR_BEAM = 0x3b6db5;
const COLOR_DEFLECTION = 0xd97a1a;
const COLOR_SUPPORT = 0x7a8593;
const COLOR_AXIS = 0xd0d7de;
const COLOR_SECTION = 0x2c5687;

const SECTION_FRACTION_ALONG_BEAM = 0.2;

const deg = (d: number) => (d * Math.PI) / 180;

// Drag tuning: time-constant of the input low-pass (smoothing) and of the
// post-release rotational decay. Larger constants = less lag / faster stop.
const SMOOTH_K = 18;   // ~55 ms lag during drag
const DECAY_K = 16;    // ~60 ms inertia time constant
const STOP_VEL = 0.1;  // rad/s — threshold to end the animation loop
const YAW_PER_PX = 1 / 150;

export class Scene {
  private renderer: THREE.WebGLRenderer;
  private root: THREE.Scene;
  private camera: THREE.OrthographicCamera;
  private content: THREE.Group;
  private axes: THREE.Group;
  private currentL: number;
  private yaw: number;
  private pitch: number;
  private targetYaw: number;
  private yawVelocity = 0;
  private dragging = false;
  private lastPointerX = 0;
  private lastFrameTime = 0;
  private animHandle = 0;

  constructor(canvas: HTMLCanvasElement, initialL: number) {
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.root = new THREE.Scene();
    this.root.background = new THREE.Color(COLOR_BG);
    this.camera = new THREE.OrthographicCamera(-1, 1, 1, -1, -10000, 10000);
    this.content = new THREE.Group();
    this.axes = new THREE.Group();
    this.root.add(this.content);
    this.root.add(this.axes);
    this.currentL = initialL;
    this.yaw = deg(ISO_YAW_DEG);
    this.pitch = deg(ISO_PITCH_DEG);
    this.targetYaw = this.yaw;

    const ro = new ResizeObserver(() => this.onResize());
    ro.observe(canvas);

    this.installDrag(canvas);
  }

  private installDrag(canvas: HTMLCanvasElement) {
    canvas.addEventListener('pointerdown', (e) => {
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

    const end = () => {
      if (!this.dragging) return;
      this.dragging = false;
      canvas.classList.remove('dragging');
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
        const alpha = 1 - Math.exp(-dt * SMOOTH_K);
        this.yaw += (this.targetYaw - this.yaw) * alpha;
        this.yawVelocity = (this.yaw - prevYaw) / Math.max(dt, 1e-3);
      } else {
        this.yaw += this.yawVelocity * dt;
        this.targetYaw = this.yaw;
        this.yawVelocity *= Math.exp(-dt * DECAY_K);
        if (Math.abs(this.yawVelocity) < STOP_VEL) this.yawVelocity = 0;
      }

      this.refresh();

      const settled = !this.dragging
        && this.yawVelocity === 0
        && Math.abs(this.targetYaw - this.yaw) < 1e-4;
      if (settled) {
        this.animHandle = 0;
        return;
      }
      this.animHandle = requestAnimationFrame(tick);
    };
    this.animHandle = requestAnimationFrame(tick);
  }

  update(s: BeamState, defl: Deflection, displayScale: number, section: CrossSection) {
    this.currentL = s.L_mm;
    this.rebuildContent(s, defl, displayScale, section);
    this.rebuildAxes(s.L_mm);
    this.refresh();
  }

  private rebuildContent(
    s: BeamState,
    defl: Deflection,
    displayScale: number,
    section: CrossSection,
  ) {
    disposeChildren(this.content);
    const L = s.L_mm;
    const r = L / 80;

    // Undeformed beam: thin translucent cylinder along +X from x=0 to x=L.
    const beamGeom = new THREE.CylinderGeometry(r, r, L, 24);
    beamGeom.rotateZ(Math.PI / 2);
    beamGeom.translate(L / 2, 0, 0);
    const beamMat = new THREE.MeshBasicMaterial({
      color: COLOR_BEAM,
      transparent: true,
      opacity: 0.28,
      depthWrite: false,
    });
    this.content.add(new THREE.Mesh(beamGeom, beamMat));

    // Supports.
    const supportMat = new THREE.MeshBasicMaterial({ color: COLOR_SUPPORT });
    if (s.beamType === 'cantilever') {
      const wall = new THREE.Mesh(
        new THREE.BoxGeometry(L / 40, L / 3, L / 3),
        supportMat,
      );
      wall.position.set(-L / 80, 0, 0);
      this.content.add(wall);
    } else {
      const pinH = L / 15;
      const pinR = L / 28;
      const pinGeom = new THREE.ConeGeometry(pinR, pinH, 4);
      pinGeom.rotateX(Math.PI / 2);
      pinGeom.translate(0, 0, -pinH / 2);
      const pin1 = new THREE.Mesh(pinGeom, supportMat);
      const pin2 = new THREE.Mesh(pinGeom, supportMat);
      pin2.position.x = L;
      this.content.add(pin1, pin2);
    }

    // Deflection ellipse at the load point: in YZ plane, semi-axes (ax along Y,
    // ay along Z). Force can point in any transverse direction with the same
    // magnitude, so the deflection traces this closed locus.
    const loadX = s.beamType === 'cantilever' ? L : L / 2;
    const ax = defl.ax_mm * displayScale;
    const ay = defl.ay_mm * displayScale;
    const tubeR = L / 220;
    if (defl.peak_mm > 0 && Math.max(ax, ay) > tubeR * 1.5) {
      const N = 96;
      const pts: THREE.Vector3[] = [];
      for (let i = 0; i < N; i++) {
        const th = (i / N) * 2 * Math.PI;
        pts.push(new THREE.Vector3(loadX, ax * Math.cos(th), ay * Math.sin(th)));
      }
      const curve = new THREE.CatmullRomCurve3(pts, true);
      const geom = new THREE.TubeGeometry(curve, N * 2, tubeR, 6, true);
      const mat = new THREE.MeshBasicMaterial({ color: COLOR_DEFLECTION });
      this.content.add(new THREE.Mesh(geom, mat));
    }

    // Cross-section: small extruded slab at x ≈ 0.2 L, in actual mm so the
    // user gets a sense of section-vs-length proportion.
    this.addCrossSection(SECTION_FRACTION_ALONG_BEAM * L, L, section);
  }

  private addCrossSection(xSec: number, L: number, section: CrossSection) {
    if (section.type === 'unreachable') return;
    const eps = L * 0.04;
    const mat = new THREE.MeshBasicMaterial({ color: COLOR_SECTION });

    if (section.type === 'cruciform') {
      const { w_mm, h_mm, t_mm } = section;
      const horiz = new THREE.Mesh(new THREE.BoxGeometry(eps, w_mm, t_mm), mat);
      const vert = new THREE.Mesh(new THREE.BoxGeometry(eps, t_mm, h_mm), mat);
      horiz.position.x = xSec;
      vert.position.x = xSec;
      this.content.add(horiz, vert);
    } else if (section.type === 'hollowBox') {
      const { W_mm, H_mm, t_mm } = section;
      const top = new THREE.Mesh(new THREE.BoxGeometry(eps, W_mm, t_mm), mat);
      top.position.set(xSec, 0, H_mm / 2 - t_mm / 2);
      const bot = new THREE.Mesh(new THREE.BoxGeometry(eps, W_mm, t_mm), mat);
      bot.position.set(xSec, 0, -H_mm / 2 + t_mm / 2);
      const inner = H_mm - 2 * t_mm;
      const left = new THREE.Mesh(new THREE.BoxGeometry(eps, t_mm, inner), mat);
      left.position.set(xSec, -W_mm / 2 + t_mm / 2, 0);
      const right = new THREE.Mesh(new THREE.BoxGeometry(eps, t_mm, inner), mat);
      right.position.set(xSec, W_mm / 2 - t_mm / 2, 0);
      this.content.add(top, bot, left, right);
    } else {
      const { b_mm, h_mm } = section;
      const mesh = new THREE.Mesh(new THREE.BoxGeometry(eps, b_mm, h_mm), mat);
      mesh.position.x = xSec;
      this.content.add(mesh);
    }
  }

  private rebuildAxes(L: number) {
    disposeChildren(this.axes);
    const mat = new THREE.LineBasicMaterial({ color: COLOR_AXIS });
    const positions = new Float32Array([
      -L * 0.1, 0, 0,
       L * 1.1, 0, 0,
    ]);
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    this.axes.add(new THREE.Line(geom, mat));
  }

  private refresh() {
    this.updateCamera();
    this.renderer.render(this.root, this.camera);
  }

  private updateCamera() {
    const L = this.currentL;
    const target = new THREE.Vector3(L / 2, 0, 0);
    const elev = -this.pitch;
    const ce = Math.cos(elev);
    const se = Math.sin(elev);
    const sy = Math.sin(this.yaw);
    const cy = Math.cos(this.yaw);
    const r = 2 * L;
    this.camera.position.set(
      target.x + r * ce * sy,
      target.y - r * ce * cy,
      target.z + r * se,
    );
    if (Math.abs(ce) < 0.01) {
      this.camera.up.set(-sy, cy, 0);
    } else {
      this.camera.up.set(0, 0, 1);
    }
    this.camera.lookAt(target);

    const canvas = this.renderer.domElement;
    const w = canvas.clientWidth || 1;
    const h = canvas.clientHeight || 1;
    const aspect = w / h;
    const halfBase = L * 0.6;
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
    this.camera.near = -10 * L;
    this.camera.far = 10 * L;
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
      material?: THREE.Material | THREE.Material[];
    };
    if (o.geometry) o.geometry.dispose();
    if (o.material) {
      if (Array.isArray(o.material)) o.material.forEach((m) => m.dispose());
      else o.material.dispose();
    }
  });
  group.clear();
}
