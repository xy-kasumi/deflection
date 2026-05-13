import * as THREE from 'three';
import type { BeamNode } from '../walker';
import { COLOR, VU } from './tokens';

// Builders for the chain itself: beam capsules + (in editor focus mode) joints,
// walker hints, attachment markers, plus the root-beam clamp markers that stay
// visible in both modes. Pure builders — no scene-graph mutation; the caller
// adds the returned meshes to its group. Also computes the chain's bbox so the
// caller can derive ortho zoom from it.
//
// All sizes are expressed as multiples of the visual unit u (= rod diameter,
// computed by the caller from the chain's average beam length). Color tokens
// live in tokens.ts; this file should not name hex literals.

export interface ChainOpts {
  supportKind: 'single' | 'both' | undefined;
  focused: boolean;
  currentBeamIx: number | null;
  u: number;
}

export interface ChainBuildResult {
  meshes: THREE.Object3D[];
  bbox: THREE.Box3;
}

export function buildChain(beams: BeamNode[], opts: ChainOpts): ChainBuildResult {
  const { supportKind, focused, currentBeamIx, u } = opts;

  const rodR     = u * VU.rodR;
  const jointR   = u * VU.jointR;
  const attachR  = u * VU.attachR;
  const clampHalf = u * VU.clampHalf;
  const walkerHintOffset = u * VU.walkerHintOffset;

  const jointGeom = new THREE.SphereGeometry(jointR, 12, 8);
  const clampGeom = new THREE.BoxGeometry(clampHalf * 2, clampHalf * 2, clampHalf * 2);
  const attachGeom = new THREE.SphereGeometry(attachR, 10, 6);
  const jointMat = new THREE.MeshBasicMaterial({ color: COLOR.joint });
  const clampMat = new THREE.MeshBasicMaterial({ color: COLOR.clamp });
  const attachMat = new THREE.MeshBasicMaterial({ color: COLOR.attachment });

  const meshes: THREE.Object3D[] = [];
  const bbox = new THREE.Box3();
  bbox.makeEmpty();

  for (let i = 0; i < beams.length; i++) {
    const b = beams[i]!;
    const start = new THREE.Vector3(...b.startFrame.origin);
    const fwd = new THREE.Vector3(...b.startFrame.fwd);
    const end = start.clone().add(fwd.clone().multiplyScalar(b.length_mm));
    const isCurrent = focused && currentBeamIx === i;

    // Capsule = cylinder + hemispherical end caps in one geometry. Trim the
    // cylinder portion by 2r so the visual span (caps included) matches
    // b.length_mm exactly. Local axis is +Y; rotate to align with beam +fwd.
    const cylPart = Math.max(rodR * 0.01, b.length_mm - 2 * rodR);
    const geom = new THREE.CapsuleGeometry(rodR, cylPart, 6, 16);
    geom.translate(0, b.length_mm / 2, 0);
    const beamMat = new THREE.MeshBasicMaterial({
      color: isCurrent ? COLOR.beamCurrent : COLOR.beam,
      transparent: true,
      opacity: 0.32,
      depthWrite: false,
    });
    const mesh = new THREE.Mesh(geom, beamMat);
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), fwd);
    mesh.position.copy(start);
    meshes.push(mesh);

    // Structural decorations (joints, loads, walker hints) only in editing mode.
    // Inspection mode keeps just the beam skeleton + clamps + deflection visuals.
    if (focused) {
      // Non-root beams: sphere at the parent-attachment point (this beam's start).
      if (i > 0) {
        const joint = new THREE.Mesh(jointGeom, jointMat);
        joint.position.copy(start);
        meshes.push(joint);
      }

      // Walker hint: sphere offset along walker-up at the beam's start with a
      // faint foot dropping orthogonally to the centerline. Surfaces the
      // section-frame orientation. Green on the editor's current beam.
      const hintColor = isCurrent ? COLOR.walkerHintCurrent : COLOR.beam;
      const upVec = new THREE.Vector3(...b.startFrame.up);
      const fwdOffset = b.length_mm * 0.08;
      const walkerHintGeom = new THREE.SphereGeometry(attachR * 0.9, 10, 6);
      const walkerHint = new THREE.Mesh(
        walkerHintGeom,
        new THREE.MeshBasicMaterial({ color: hintColor }),
      );
      walkerHint.position
        .copy(start)
        .add(fwd.clone().multiplyScalar(fwdOffset))
        .add(upVec.clone().multiplyScalar(walkerHintOffset));
      meshes.push(walkerHint);

      const footAnchor = start.clone().add(fwd.clone().multiplyScalar(fwdOffset));
      const footGeom = new THREE.BufferGeometry().setFromPoints([
        footAnchor,
        walkerHint.position.clone(),
      ]);
      const footMat = new THREE.LineBasicMaterial({
        color: hintColor,
        transparent: true,
        opacity: 0.35,
        depthWrite: false,
      });
      meshes.push(new THREE.Line(footGeom, footMat));

      // Attachment markers along the beam axis (load locations).
      for (const att of b.attachmentOffsets) {
        const pos = start.clone().add(fwd.clone().multiplyScalar(att.local_mm));
        const dot = new THREE.Mesh(attachGeom, attachMat);
        dot.position.copy(pos);
        meshes.push(dot);
      }
    }

    bbox.expandByPoint(start);
    bbox.expandByPoint(end);
  }

  // Clamp markers on the root beam: start always; end under support(both).
  // These stay visible in both modes — they denote the world-origin reference.
  if (supportKind) {
    const root = beams[0]!;
    const rStart = new THREE.Vector3(...root.startFrame.origin);
    const rFwd = new THREE.Vector3(...root.startFrame.fwd);
    const startClamp = new THREE.Mesh(clampGeom, clampMat);
    startClamp.position.copy(rStart);
    meshes.push(startClamp);
    if (supportKind === 'both') {
      const endClamp = new THREE.Mesh(clampGeom, clampMat);
      endClamp.position.copy(rStart).add(rFwd.multiplyScalar(root.length_mm));
      meshes.push(endClamp);
    }
  }

  return { meshes, bbox };
}
