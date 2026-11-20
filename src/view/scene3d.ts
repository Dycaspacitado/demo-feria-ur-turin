/**
 * Vista 3D con Three.js. Solo LEE el estado de la simulación: nunca decide movimientos.
 * Todo el mundo está en milímetros con Z hacia arriba (igual que los controladores de los robots).
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/examples/jsm/environments/RoomEnvironment.js';
import { RoundedBoxGeometry } from 'three/examples/jsm/geometries/RoundedBoxGeometry.js';
import { type Pose, type Vec3, add, dot, norm, normalize, scale, sub, poseMul } from '../kin/math';
import { type CustomerActor, type RobotActor, type Scenario, type SimObject, type Simulation } from '../sim/engine';
import { GRIPPER } from '../sim/gripper';
import { robotSpheres } from '../sim/collision';
import { TABLE } from '../sim/common';
import { KEYCHAIN, STAND } from '../sim/scenarioKeychain';
import { BUTTON, SERVICE_STAND } from '../sim/scenarioService';
import { CUBE, FIXTURE, TRAYS } from '../sim/scenarioTower';
import { floorTexture, hazardTexture, textTexture } from './textures';

const m4 = new THREE.Matrix4();
function setPose(o: THREE.Object3D, P: Pose) {
  const R = P.R, p = P.p;
  m4.set(R[0], R[1], R[2], p[0], R[3], R[4], R[5], p[1], R[6], R[7], R[8], p[2], 0, 0, 0, 1);
  o.matrix.copy(m4);
  o.matrixWorldNeedsUpdate = true;
}

function decode(b64: string): ArrayBuffer {
  const bin = atob(b64);
  const buf = new ArrayBuffer(bin.length);
  const u8 = new Uint8Array(buf);
  for (let i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
  return buf;
}

const ROBOT_ACCENT: Record<string, string> = { ur: '#3fc9d4', turin: '#e9c41c' };
const geoCache = new Map<string, THREE.BufferGeometry>();
const cachedGeos = new WeakSet<THREE.BufferGeometry>();

interface RobotView { actor: RobotActor; links: THREE.Group[]; gripper: THREE.Group; fingers: THREE.Group[]; trail: THREE.Line; trailPts: Float32Array; trailN: number }

export const CAMERA_PRESETS = {
  general: { pos: [-1750, -2900, 1950] as Vec3, target: [60, -120, 150] as Vec3, label: 'General' },
  frontal: { pos: [0, -2600, 900] as Vec3, target: [0, -40, 200] as Vec3, label: 'Frontal' },
  superior: { pos: [0, -350, 2700] as Vec3, target: [0, -60, 0] as Vec3, label: 'Superior' },
  detalle: { pos: [420, -760, 640] as Vec3, target: [0, 0, 280] as Vec3, label: 'Detalle' },
};
export type CameraPreset = keyof typeof CAMERA_PRESETS;

export class Scene3D {
  readonly renderer: THREE.WebGLRenderer;
  readonly scene = new THREE.Scene();
  readonly camera: THREE.PerspectiveCamera;
  readonly controls: OrbitControls;
  private cell = new THREE.Group();
  private robots: RobotView[] = [];
  private objects = new Map<string, THREE.Object3D>();
  private customer: { actor: CustomerActor; group: THREE.Group; arm: THREE.Group; upper: THREE.Mesh; fore: THREE.Mesh; hand: THREE.Group; fingers: THREE.Mesh; legs: THREE.Group[]; leftArm: THREE.Group } | null = null;
  private button: { cap: THREE.Mesh; mat: THREE.MeshStandardMaterial; ring: THREE.MeshStandardMaterial } | null = null;
  private envelope = new THREE.Group();
  private lamp: { green: THREE.MeshStandardMaterial; amber: THREE.MeshStandardMaterial; red: THREE.MeshStandardMaterial } | null = null;
  private camAnim: { from: THREE.Vector3; to: THREE.Vector3; tf: THREE.Vector3; tt: THREE.Vector3; t: number } | null = null;
  showEnvelopes = false;
  showTrails = true;
  detailFocus: { pos: Vec3; target: Vec3 } = { pos: CAMERA_PRESETS.detalle.pos, target: CAMERA_PRESETS.detalle.target };

  constructor(readonly host: HTMLElement) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    this.renderer.setPixelRatio(Math.min(2, window.devicePixelRatio));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.0;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    host.appendChild(this.renderer.domElement);

    this.camera = new THREE.PerspectiveCamera(38, 1, 10, 60000);
    this.camera.up.set(0, 0, 1);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.minDistance = 250;
    this.controls.maxDistance = 9000;
    this.controls.maxPolarAngle = Math.PI * 0.495;
    this.controls.addEventListener('start', () => (this.camAnim = null));
    this.setCamera('general', false);

    const pmrem = new THREE.PMREMGenerator(this.renderer);
    this.scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
    this.scene.environmentIntensity = 0.55;
    this.scene.background = new THREE.Color('#e9edf1');
    this.scene.fog = new THREE.Fog('#e9edf1', 7000, 16000);

    this.buildHall();
    this.scene.add(this.cell, this.envelope);
    new ResizeObserver(() => this.resize()).observe(host);
    this.resize();
  }

  resize() {
    const w = this.host.clientWidth || 800, h = this.host.clientHeight || 600;
    this.renderer.setSize(w, h, false);
    this.renderer.domElement.style.width = '100%';
    this.renderer.domElement.style.height = '100%';
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  setCamera(name: CameraPreset, animate = true) {
    const p = name === 'detalle' ? this.detailFocus : CAMERA_PRESETS[name];
    const to = new THREE.Vector3(...p.pos), tt = new THREE.Vector3(...p.target);
    // en visores angostos (vertical o celular) se aleja la cámara para que entre toda la celda
    const k = Math.min(1.8, Math.max(1, 1.05 / this.camera.aspect));
    to.sub(tt).multiplyScalar(k).add(tt);
    if (!animate) {
      this.camera.position.copy(to);
      this.controls.target.copy(tt);
      this.controls.update();
      return;
    }
    this.camAnim = { from: this.camera.position.clone(), to, tf: this.controls.target.clone(), tt, t: 0 };
  }

  // ------------------------------------------------------------------ nave / stand fijo
  private buildHall() {
    const hemi = new THREE.HemisphereLight('#ffffff', '#8b9095', 0.9);
    hemi.position.set(0, 0, 1);
    this.scene.add(hemi);
    const key = new THREE.DirectionalLight('#ffffff', 2.1);
    key.position.set(1800, -2600, 3800);
    key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    key.shadow.camera.left = -1700; key.shadow.camera.right = 1700;
    key.shadow.camera.top = 1700; key.shadow.camera.bottom = -1700;
    key.shadow.camera.near = 1000; key.shadow.camera.far = 9000;
    key.shadow.bias = -0.0004;
    key.shadow.normalBias = 1.5;
    this.scene.add(key);
    const fill = new THREE.DirectionalLight('#dfe8ff', 0.6);
    fill.position.set(-2500, 1500, 2000);
    this.scene.add(fill);

    // piso
    const ft = floorTexture();
    ft.repeat.set(24, 24);
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(12000, 12000), new THREE.MeshStandardMaterial({ map: ft, roughness: 0.62, metalness: 0.0 }));
    floor.position.z = TABLE.floor;
    floor.receiveShadow = true;
    this.scene.add(floor);
    // cinta de seguridad alrededor de la celda
    const ht = hazardTexture();
    const tapeMat = new THREE.MeshStandardMaterial({ map: ht, roughness: 0.6 });
    const addTape = (x: number, y: number, len: number, rot: number) => {
      const t = ht.clone(); t.repeat.set(len / 120, 1); t.needsUpdate = true;
      const m = new THREE.Mesh(new THREE.PlaneGeometry(len, 60), tapeMat.clone());
      (m.material as THREE.MeshStandardMaterial).map = t;
      m.position.set(x, y, TABLE.floor + 1); m.rotation.z = rot; m.receiveShadow = true;
      this.scene.add(m);
    };
    addTape(0, -1320, 2700, 0); addTape(0, 1150, 2700, 0); addTape(-1320, -85, 2470, Math.PI / 2); addTape(1320, -85, 2470, Math.PI / 2);

    // mesa: tablero de aluminio + perfiles
    const top = new THREE.Mesh(
      new RoundedBoxGeometry(TABLE.x1 - TABLE.x0, TABLE.y1 - TABLE.y0, TABLE.thick, 3, 6),
      new THREE.MeshStandardMaterial({ color: '#d6dade', roughness: 0.38, metalness: 0.55 }),
    );
    top.position.set(0, 0, -TABLE.thick / 2);
    top.receiveShadow = true; top.castShadow = true;
    this.scene.add(top);
    const profMat = new THREE.MeshStandardMaterial({ color: '#9aa3ab', roughness: 0.35, metalness: 0.8 });
    const legH = -TABLE.floor - TABLE.thick;
    for (const x of [TABLE.x0 + 40, TABLE.x1 - 40]) for (const y of [TABLE.y0 + 40, TABLE.y1 - 40]) {
      const leg = new THREE.Mesh(new THREE.BoxGeometry(45, 45, legH), profMat);
      leg.position.set(x, y, TABLE.floor + legH / 2); leg.castShadow = true;
      this.scene.add(leg);
    }
    for (const z of [TABLE.floor + 140, -TABLE.thick - 40]) {
      for (const y of [TABLE.y0 + 40, TABLE.y1 - 40]) {
        const b = new THREE.Mesh(new THREE.BoxGeometry(TABLE.x1 - TABLE.x0 - 80, 45, 45), profMat);
        b.position.set(0, y, z); this.scene.add(b);
      }
      for (const x of [TABLE.x0 + 40, TABLE.x1 - 40]) {
        const b = new THREE.Mesh(new THREE.BoxGeometry(45, TABLE.y1 - TABLE.y0 - 80, 45), profMat);
        b.position.set(x, 0, z); this.scene.add(b);
      }
    }
    // faldón frontal con la marca
    const skirt = new THREE.Mesh(new THREE.PlaneGeometry(1500, 160), new THREE.MeshStandardMaterial({
      map: textTexture('GRUPRO  ·  ROBÓTICA COLABORATIVA', { w: 2048, h: 220, bg: '#14315c', fg: '#ffffff', size: 104 }), roughness: 0.6,
    }));
    skirt.rotation.x = Math.PI / 2;
    skirt.position.set(0, TABLE.y0 - 24, -TABLE.thick - 110);
    this.scene.add(skirt);

    // panel de fondo del stand
    const wall = new THREE.Mesh(new THREE.BoxGeometry(4200, 40, 2600), new THREE.MeshStandardMaterial({ color: '#f4f6f8', roughness: 0.9 }));
    wall.position.set(0, 1900, TABLE.floor + 1300); wall.receiveShadow = true;
    this.scene.add(wall);
    const sign = new THREE.Mesh(new THREE.PlaneGeometry(2600, 520), new THREE.MeshStandardMaterial({
      map: textTexture('GRUPRO', { w: 2048, h: 410, bg: '#14315c', fg: '#ffffff', size: 230, sub: 'Automatización industrial · Cobots UR + Turin' }), roughness: 0.55,
    }));
    sign.rotation.x = Math.PI / 2;
    sign.position.set(0, 1878, TABLE.floor + 1900);
    this.scene.add(sign);
    const stripe = new THREE.Mesh(new THREE.BoxGeometry(4200, 44, 60), new THREE.MeshStandardMaterial({ color: '#f2c200', roughness: 0.5 }));
    stripe.position.set(0, 1898, TABLE.floor + 30);
    this.scene.add(stripe);
  }

  // ------------------------------------------------------------------ celda de cada simulación
  load(sc: Scenario) {
    this.disposeCell();
    this.robots = [];
    this.objects.clear();
    this.customer = null;
    for (const r of sc.robots) this.robots.push(this.buildRobot(r));
    for (const o of sc.objects) {
      const g = o.kind === 'keychain' ? this.buildKeychain() : this.buildCube(o);
      g.matrixAutoUpdate = false;
      this.cell.add(g);
      this.objects.set(o.id, g);
    }
    this.button = null;
    if (sc.id === 'llavero') this.buildKeychainFixtures(STAND, [0, 150], [520, -440]);
    else this.buildTowerFixtures();
    if (sc.id === 'servicio') {
      this.buildKeychainFixtures(SERVICE_STAND, [-80, 220], [560, -430]);
      this.buildButton();
    }
    if (sc.customer) this.buildCustomer(sc.customer);
    this.buildStackLight();
    this.detailFocus = sc.id === 'llavero'
      ? { pos: [430, -620, 720], target: [0, 120, 320] }
      : sc.id === 'servicio'
        ? { pos: [1750, 650, 1150], target: [300, -380, 230] }
        : { pos: [330, -1050, 520], target: [0, -120, 220] };
  }

  private disposeCell() {
    // libera geometrías (salvo las mallas de los robots, que se reutilizan), materiales y texturas:
    // en modo feria la celda se reconstruye en cada ciclo y no debe acumular memoria de video
    const mats = new Set<THREE.Material>();
    this.cell.traverse((o) => {
      if (o instanceof THREE.Mesh || o instanceof THREE.Line || o instanceof THREE.LineSegments) {
        if (!cachedGeos.has(o.geometry)) o.geometry.dispose();
        for (const m of Array.isArray(o.material) ? o.material : [o.material]) mats.add(m);
      }
    });
    for (const m of mats) {
      const map = (m as THREE.MeshStandardMaterial).map;
      if (map) map.dispose();
      m.dispose();
    }
    this.envelope.traverse((o) => { if (o instanceof THREE.InstancedMesh) { o.geometry.dispose(); (o.material as THREE.Material).dispose(); o.dispose(); } });
    this.cell.clear();
    this.envelope.clear();
  }

  private robotGeometry(key: string, positions: string, indices: string, index32: boolean): THREE.BufferGeometry {
    const cached = geoCache.get(key);
    if (cached) return cached;
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(decode(positions)), 3));
    g.setIndex(new THREE.BufferAttribute(index32 ? new Uint32Array(decode(indices)) : new Uint16Array(decode(indices)), 1));
    g.computeVertexNormals();
    geoCache.set(key, g);
    cachedGeos.add(g);
    return g;
  }

  private buildRobot(r: RobotActor): RobotView {
    const data = r.kin.data;
    const links: THREE.Group[] = [];
    const mats = new Map<string, THREE.MeshStandardMaterial>();
    const mat = (color: string) => {
      let m = mats.get(color);
      if (!m) {
        const c = new THREE.Color(color);
        const hsl = { h: 0, s: 0, l: 0 }; c.getHSL(hsl);
        const metallic = hsl.s < 0.12 && hsl.l > 0.45;
        m = new THREE.MeshStandardMaterial({ color: c, roughness: metallic ? 0.32 : 0.42, metalness: metallic ? 0.55 : 0.12, envMapIntensity: 0.9 });
        mats.set(color, m);
      }
      return m;
    };
    data.links.forEach((l, i) => {
      const g = new THREE.Group();
      g.matrixAutoUpdate = false;
      for (const p of l.parts) {
        const mesh = new THREE.Mesh(this.robotGeometry(`${data.name}/${p.name}`, p.positions, p.indices, p.index32), mat(p.color));
        mesh.castShadow = true; mesh.receiveShadow = true;
        g.add(mesh);
      }
      g.name = `${data.name}_link${i}`;
      links.push(g);
      this.cell.add(g);
    });
    // placa de montaje sobre la mesa y cartel con el nombre
    const plate = new THREE.Mesh(new THREE.CylinderGeometry(105, 105, 8, 48), new THREE.MeshStandardMaterial({ color: '#59616a', roughness: 0.4, metalness: 0.7 }));
    plate.rotation.x = Math.PI / 2;
    plate.position.set(r.kin.base.p[0], r.kin.base.p[1], -2);
    plate.receiveShadow = true;
    this.cell.add(plate);
    const tag = new THREE.Mesh(new THREE.PlaneGeometry(220, 55), new THREE.MeshStandardMaterial({
      map: textTexture(r.label.toUpperCase(), { w: 512, h: 128, bg: '#1d2229', fg: r.id === 'ur' ? '#7fe3ea' : '#f2d34a', size: 64 }), roughness: 0.5,
    }));
    tag.rotation.x = Math.PI / 2;
    tag.position.set(r.kin.base.p[0], TABLE.y0 - 1.5, -TABLE.thick / 2);
    this.cell.add(tag);

    const { gripper, fingers } = this.buildGripper(ROBOT_ACCENT[r.id] ?? '#cccccc');
    this.cell.add(gripper);

    const max = 1400;
    const trailPts = new Float32Array(max * 3);
    const tg = new THREE.BufferGeometry();
    tg.setAttribute('position', new THREE.BufferAttribute(trailPts, 3));
    tg.setDrawRange(0, 0);
    const trail = new THREE.Line(tg, new THREE.LineBasicMaterial({ color: r.id === 'ur' ? '#14a6b3' : '#c99a00', transparent: true, opacity: 0.85 }));
    trail.frustumCulled = false;
    this.cell.add(trail);
    return { actor: r, links, gripper, fingers, trail, trailPts, trailN: 0 };
  }

  private buildGripper(accent: string) {
    const gripper = new THREE.Group();
    gripper.matrixAutoUpdate = false;
    const dark = new THREE.MeshStandardMaterial({ color: '#2a2e33', roughness: 0.45, metalness: 0.35 });
    const alu = new THREE.MeshStandardMaterial({ color: '#c3c8cd', roughness: 0.3, metalness: 0.85 });
    const acc = new THREE.MeshStandardMaterial({ color: accent, roughness: 0.4, metalness: 0.15 });
    const ad = new THREE.Mesh(new THREE.CylinderGeometry(GRIPPER.adapter.r, GRIPPER.adapter.r, 10, 40), alu);
    ad.rotation.x = Math.PI / 2; ad.position.z = 5;
    const b = GRIPPER.body;
    const body = new THREE.Mesh(new RoundedBoxGeometry(b.hx * 2, b.hy * 2, b.z1 - b.z0, 3, 5), dark);
    body.position.z = (b.z0 + b.z1) / 2;
    const side1 = new THREE.Mesh(new THREE.BoxGeometry(2, b.hy * 1.6, (b.z1 - b.z0) * 0.55), acc);
    side1.position.set(b.hx + 1, 0, body.position.z);
    const side2 = side1.clone(); side2.position.x = -b.hx - 1;
    const rail = new THREE.Mesh(new THREE.BoxGeometry(b.hx * 1.2, b.hy * 2 + 8, 6), alu);
    rail.position.z = b.z1 + 2;
    gripper.add(ad, body, side1, side2, rail);
    const fingers: THREE.Group[] = [];
    const f = GRIPPER.finger;
    for (const sgn of [1, -1]) {
      const fg = new THREE.Group();
      const fm = new THREE.Mesh(new RoundedBoxGeometry(f.hx * 2, f.t, f.z1 - f.z0, 2, 2), alu);
      fm.position.set(0, (sgn * f.t) / 2, (f.z0 + f.z1) / 2);
      const pad = new THREE.Mesh(new THREE.BoxGeometry(f.hx * 2 - 2, 1.6, 26), new THREE.MeshStandardMaterial({ color: '#222', roughness: 0.9 }));
      pad.position.set(0, sgn * 0.8, f.z1 - 14);
      const car = new THREE.Mesh(new THREE.BoxGeometry(f.hx * 2 + 4, f.t + 8, 14), dark);
      car.position.set(0, (sgn * (f.t + 8)) / 2, f.z0 + 4);
      fg.add(fm, pad, car);
      fg.userData.sign = sgn;
      fingers.push(fg);
      gripper.add(fg);
    }
    gripper.traverse((o) => { if (o instanceof THREE.Mesh) { o.castShadow = true; o.receiveShadow = true; } });
    return { gripper, fingers };
  }

  private buildKeychain(): THREE.Group {
    const g = new THREE.Group();
    const t = KEYCHAIN.tag;
    const face = textTexture('GRUPRO', { w: 512, h: 224, bg: '#14315c', fg: '#ffffff', size: 118, border: '#f2c200' });
    const plain = new THREE.MeshStandardMaterial({ color: '#14315c', roughness: 0.25, metalness: 0.1 });
    const faceMat = new THREE.MeshStandardMaterial({ map: face, roughness: 0.25, metalness: 0.1 });
    // caja con las dos caras grandes (±Y) impresas con el logo
    const tagGeo = new THREE.BoxGeometry(t.hx * 2, t.hy * 2, t.hz * 2);
    fixVerticalFaceUV(tagGeo);
    const tag = new THREE.Mesh(tagGeo, [plain, plain, faceMat, faceMat, plain, plain]);
    tag.castShadow = true;
    const r = KEYCHAIN.ring;
    const ring = new THREE.Mesh(new THREE.TorusGeometry(r.r, r.tube, 12, 40), new THREE.MeshStandardMaterial({ color: '#d9dde1', roughness: 0.2, metalness: 1 }));
    ring.rotation.x = Math.PI / 2;
    ring.position.set(r.c[0], r.c[1], r.c[2]);
    ring.castShadow = true;
    g.add(tag, ring);
    return g;
  }

  private buildCube(o: SimObject): THREE.Group {
    const g = new THREE.Group();
    const plain = new THREE.MeshStandardMaterial({ color: '#eceff2', roughness: 0.45, metalness: 0.05 });
    const letter = new THREE.MeshStandardMaterial({
      map: textTexture(o.letter ?? '?', { w: 256, h: 256, bg: '#eceff2', fg: '#14315c', size: 190, border: '#f2c200' }), roughness: 0.45,
    });
    const back = new THREE.MeshStandardMaterial({
      map: textTexture(o.letter ?? '?', { w: 256, h: 256, bg: '#eceff2', fg: '#9aa6b2', size: 150 }), roughness: 0.45,
    });
    // orden de caras de BoxGeometry: +X, -X, +Y, -Y, +Z, -Z  → letra en la cara -Y (hacia la cámara frontal)
    const geo = new THREE.BoxGeometry(CUBE - 0.6, CUBE - 0.6, CUBE - 0.6);
    fixVerticalFaceUV(geo);
    const m = new THREE.Mesh(geo, [plain, plain, back, letter, plain, plain]);
    m.castShadow = true; m.receiveShadow = true;
    const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geo), new THREE.LineBasicMaterial({ color: '#8f9aa5' }));
    g.add(m, edges);
    return g;
  }

  private buildKeychainFixtures(stand: { c: Vec3; h: Vec3 }, xfer: [number, number], deliver: [number, number]) {
    const st = new THREE.Mesh(new RoundedBoxGeometry(stand.h[0] * 2, stand.h[1] * 2, stand.h[2] * 2, 2, 3), new THREE.MeshStandardMaterial({ color: '#1e2329', roughness: 0.5, metalness: 0.3 }));
    st.position.set(...stand.c);
    st.castShadow = true; st.receiveShadow = true;
    const slot = new THREE.Mesh(new THREE.BoxGeometry(84, 7, 2), new THREE.MeshStandardMaterial({ color: '#000' }));
    slot.position.set(stand.c[0], stand.c[1], stand.c[2] + stand.h[2] + 0.5);
    this.cell.add(st, slot);
    // marca de la zona de transferencia sobre la mesa
    const ring = new THREE.Mesh(new THREE.RingGeometry(120, 128, 64), new THREE.MeshBasicMaterial({ color: '#f2c200', transparent: true, opacity: 0.8 }));
    ring.position.set(xfer[0], xfer[1], 0.6);
    const label = new THREE.Mesh(new THREE.PlaneGeometry(220, 34), new THREE.MeshBasicMaterial({ map: textTexture('ZONA DE TRANSFERENCIA', { w: 1024, h: 160, bg: '#d6dade', fg: '#3a4048', size: 92 }) }));
    label.position.set(xfer[0], xfer[1] + 150, 0.7);
    this.cell.add(ring, label);
    // cartel de entrega
    const sign = new THREE.Mesh(new THREE.PlaneGeometry(240, 40), new THREE.MeshBasicMaterial({ map: textTexture('ENTREGA AL CLIENTE', { w: 1024, h: 170, bg: '#d6dade', fg: '#14315c', size: 92 }) }));
    sign.position.set(deliver[0], deliver[1], 0.7);
    this.cell.add(sign);
  }

  /** Pedestal con el pulsador "QUIERO MI LLAVERO" para el visitante. */
  private buildButton() {
    const b = BUTTON.base, top = BUTTON.top;
    const metal = new THREE.MeshStandardMaterial({ color: '#2a2f36', roughness: 0.4, metalness: 0.6 });
    const foot = new THREE.Mesh(new THREE.CylinderGeometry(170, 190, 30, 40), metal);
    foot.rotation.x = Math.PI / 2; foot.position.set(b[0], b[1], b[2] + 15);
    const postH = top - b[2] - 60;
    const post = new THREE.Mesh(new THREE.CylinderGeometry(45, 45, postH, 24), new THREE.MeshStandardMaterial({ color: '#c3c8cd', roughness: 0.3, metalness: 0.85 }));
    post.rotation.x = Math.PI / 2; post.position.set(b[0], b[1], b[2] + 30 + postH / 2);
    const box = new THREE.Mesh(new RoundedBoxGeometry(170, 140, 60, 2, 10), new THREE.MeshStandardMaterial({ color: '#f2c200', roughness: 0.45 }));
    box.position.set(b[0], b[1], top - 30);
    const ringMat = new THREE.MeshStandardMaterial({ color: '#c9c9c9', emissive: '#ffd84a', emissiveIntensity: 0.0, roughness: 0.3 });
    const ring = new THREE.Mesh(new THREE.TorusGeometry(42, 6, 12, 40), ringMat);
    ring.position.set(b[0], b[1], top + 2);
    const mat = new THREE.MeshStandardMaterial({ color: '#d61f1f', emissive: '#ff2a2a', emissiveIntensity: 0.05, roughness: 0.35 });
    const cap = new THREE.Mesh(new THREE.SphereGeometry(38, 32, 16, 0, Math.PI * 2, 0, Math.PI / 2), mat);
    cap.rotation.x = Math.PI / 2; cap.scale.set(1, 0.55, 1);
    cap.position.set(b[0], b[1], top + 4);
    const sign = new THREE.Mesh(new THREE.PlaneGeometry(160, 52), new THREE.MeshBasicMaterial({ map: textTexture('QUIERO MI LLAVERO', { w: 768, h: 250, bg: '#14315c', fg: '#ffffff', size: 70 }) }));
    sign.rotation.x = Math.PI / 2; sign.position.set(b[0], b[1] - 71, top - 30);
    for (const m of [foot, post, box, cap]) { m.castShadow = true; m.receiveShadow = true; }
    this.cell.add(foot, post, box, ring, cap, sign);
    this.button = { cap, mat, ring: ringMat };
  }

  private buildTowerFixtures() {
    const mk = (c: Vec3, h: Vec3, color: string) => {
      const m = new THREE.Mesh(new RoundedBoxGeometry(h[0] * 2, h[1] * 2, h[2] * 2, 2, 2), new THREE.MeshStandardMaterial({ color, roughness: 0.45, metalness: 0.4 }));
      m.position.set(...c); m.receiveShadow = true; m.castShadow = true;
      this.cell.add(m);
    };
    mk(FIXTURE.c, FIXTURE.h, '#2a2f36');
    mk(TRAYS.ur.c, TRAYS.ur.h, '#3d4650');
    mk(TRAYS.turin.c, TRAYS.turin.h, '#3d4650');
    // tope en escuadra de la base de la torre
    const stop = new THREE.Mesh(new THREE.BoxGeometry(8, 74, 20), new THREE.MeshStandardMaterial({ color: '#f2c200', roughness: 0.4 }));
    stop.position.set(FIXTURE.c[0] - 34, FIXTURE.c[1] + 2, 20);
    this.cell.add(stop);
    const label = (txt: string, x: number, y: number, w = 230) => {
      const m = new THREE.Mesh(new THREE.PlaneGeometry(w, 30), new THREE.MeshBasicMaterial({ map: textTexture(txt, { w: 1024, h: 134, bg: '#d6dade', fg: '#3a4048', size: 78 }) }));
      m.position.set(x, y, 0.7);
      this.cell.add(m);
    };
    label('ALIMENTACIÓN UR5', TRAYS.ur.c[0], TRAYS.ur.c[1] - 80);
    label('ALIMENTACIÓN TURIN', TRAYS.turin.c[0], TRAYS.turin.c[1] - 80);
    label('PALETIZADO', FIXTURE.c[0], FIXTURE.c[1] - 95, 170);
    const zone = new THREE.Mesh(new THREE.RingGeometry(200, 207, 64), new THREE.MeshBasicMaterial({ color: '#d33', transparent: true, opacity: 0.55 }));
    zone.position.set(FIXTURE.c[0], FIXTURE.c[1], 0.6);
    this.cell.add(zone);
  }

  private buildStackLight() {
    const g = new THREE.Group();
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(10, 10, 260, 16), new THREE.MeshStandardMaterial({ color: '#7a828a', metalness: 0.8, roughness: 0.3 }));
    pole.rotation.x = Math.PI / 2; pole.position.z = 130;
    g.add(pole);
    const mk = (color: string, z: number) => {
      const m = new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.05, transparent: true, opacity: 0.9, roughness: 0.2 });
      const c = new THREE.Mesh(new THREE.CylinderGeometry(26, 26, 42, 24), m);
      c.rotation.x = Math.PI / 2; c.position.z = z;
      g.add(c);
      return m;
    };
    this.lamp = { red: mk('#ff3b30', 380), amber: mk('#ffb300', 336), green: mk('#2bd45a', 292) };
    g.position.set(TABLE.x0 + 70, TABLE.y1 - 70, 0);
    this.cell.add(g);
  }

  private buildCustomer(c: CustomerActor) {
    const g = new THREE.Group();
    const skin = new THREE.MeshStandardMaterial({ color: '#d9ad8c', roughness: 0.6 });
    const shirt = new THREE.MeshStandardMaterial({ color: '#2f6db5', roughness: 0.7 });
    const pants = new THREE.MeshStandardMaterial({ color: '#2a2f37', roughness: 0.8 });
    const shoe = new THREE.MeshStandardMaterial({ color: '#15171a', roughness: 0.6 });
    const cap = (r: number, len: number, mat: THREE.Material) => {
      const m = new THREE.Mesh(new THREE.CapsuleGeometry(r, len, 6, 16), mat);
      m.castShadow = true;
      return m;
    };
    // piernas articuladas en la cadera (z = 860) para poder caminar
    const legs: THREE.Group[] = [];
    for (const dx of [-95, 95]) {
      const hip = new THREE.Group();
      hip.position.set(dx, 0, 860);
      const leg = cap(68, 700, pants);
      leg.rotation.x = Math.PI / 2; leg.position.set(0, 0, -415);
      const sh = new THREE.Mesh(new RoundedBoxGeometry(110, 250, 70, 2, 25), shoe);
      sh.position.set(0, 40, -825); sh.castShadow = true;
      hip.add(leg, sh);
      legs.push(hip);
      g.add(hip);
    }
    const torso = cap(170, 380, shirt);
    torso.rotation.x = Math.PI / 2; torso.scale.set(1.2, 1, 0.72);
    torso.position.set(0, 0, 1150);
    const neck = cap(48, 60, skin);
    neck.rotation.x = Math.PI / 2; neck.position.set(0, 0, 1480);
    const head = new THREE.Mesh(new THREE.SphereGeometry(110, 32, 20), skin);
    head.scale.set(0.9, 1, 1.12); head.position.set(0, 8, 1620); head.castShadow = true;
    const hair = new THREE.Mesh(new THREE.SphereGeometry(114, 32, 16, 0, Math.PI * 2, 0, Math.PI * 0.55), new THREE.MeshStandardMaterial({ color: '#3b2a20', roughness: 0.8 }));
    hair.rotation.x = Math.PI / 2; hair.scale.set(0.92, 1.0, 1.15); hair.position.set(0, -6, 1640);
    const badge = new THREE.Mesh(new THREE.PlaneGeometry(90, 120), new THREE.MeshStandardMaterial({ map: textTexture('VISITA', { w: 256, h: 340, bg: '#ffffff', fg: '#14315c', size: 54 }) }));
    badge.rotation.x = Math.PI / 2; badge.position.set(-70, 128, 1230);
    g.add(torso, neck, head, hair, badge);
    // brazo izquierdo relajado (se balancea al caminar)
    const leftArm = new THREE.Group();
    leftArm.position.set(-190, 0, 1400);
    const la = cap(42, 520, shirt);
    la.rotation.x = Math.PI / 2; la.position.set(0, 0, -300);
    const lh = new THREE.Mesh(new THREE.SphereGeometry(45, 16, 12), skin);
    lh.position.set(-5, 0, -600);
    leftArm.add(la, lh);
    g.add(leftArm);
    this.cell.add(g);
    // brazo derecho articulado (IK de dos segmentos hacia la mano), en coordenadas del mundo
    const arm = new THREE.Group();
    const upper = cap(44, 230, shirt);
    const fore = cap(38, 210, skin);
    const hand = new THREE.Group();
    const palm = new THREE.Mesh(new RoundedBoxGeometry(80, 95, 30, 2, 12), skin);
    const fingers = new THREE.Mesh(new RoundedBoxGeometry(76, 60, 26, 2, 11), skin);
    fingers.position.set(0, 60, 0);
    palm.castShadow = fingers.castShadow = true;
    hand.add(palm, fingers);
    arm.add(upper, fore, hand);
    this.cell.add(arm);
    this.customer = { actor: c, group: g, arm, upper, fore, hand, fingers, legs, leftArm };
  }

  // ------------------------------------------------------------------ actualización por fotograma
  sync(sim: Simulation, dtReal: number) {
    for (const rv of this.robots) {
      const r = rv.actor;
      const L = r.kin.linkPoses(r.q);
      L.forEach((P, i) => setPose(rv.links[i], P));
      const F = poseMul(L[6], r.kin.M);
      setPose(rv.gripper, F);
      for (const fg of rv.fingers) fg.position.y = (fg.userData.sign as number) * (r.gripWidth / 2);
      // estela del TCP
      const tcp = r.tcp().p;
      const n = rv.trailN;
      const last = n > 0 ? [rv.trailPts[(n - 1) * 3], rv.trailPts[(n - 1) * 3 + 1], rv.trailPts[(n - 1) * 3 + 2]] : null;
      if (!last || Math.hypot(tcp[0] - last[0], tcp[1] - last[1], tcp[2] - last[2]) > 4) {
        if (n >= rv.trailPts.length / 3) { rv.trailPts.copyWithin(0, 3); rv.trailN--; }
        rv.trailPts.set(tcp, rv.trailN * 3);
        rv.trailN++;
        rv.trail.geometry.setDrawRange(0, rv.trailN);
        (rv.trail.geometry.getAttribute('position') as THREE.BufferAttribute).needsUpdate = true;
      }
      rv.trail.visible = this.showTrails;
    }
    for (const o of sim.sc.objects) setPose(this.objects.get(o.id)!, o.pose);
    if (this.customer) this.syncCustomer();
    this.syncButton(sim);
    this.syncEnvelopes(sim);
    this.syncLamp(sim);
    if (this.camAnim) {
      const a = this.camAnim;
      a.t = Math.min(1, a.t + dtReal / 1.2);
      const s = a.t * a.t * (3 - 2 * a.t);
      this.camera.position.lerpVectors(a.from, a.to, s);
      this.controls.target.lerpVectors(a.tf, a.tt, s);
      if (a.t >= 1) this.camAnim = null;
    }
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  resetTrails() { for (const rv of this.robots) { rv.trailN = 0; rv.trail.geometry.setDrawRange(0, 0); } }

  private syncCustomer() {
    const cv = this.customer!;
    const c = cv.actor;
    cv.group.visible = cv.arm.visible = c.present;
    if (!c.present) return;
    cv.group.position.set(c.feet[0], c.feet[1], c.feet[2]);
    cv.group.rotation.z = c.yaw;
    // caminata: piernas y brazo izquierdo en contrafase
    const sw = c.walking ? Math.sin(c.stride / 115) * 0.42 : 0;
    cv.legs[0].rotation.x = sw;
    cv.legs[1].rotation.x = -sw;
    cv.leftArm.rotation.x = -sw * 0.7;
    cv.group.position.z = c.feet[2] + (c.walking ? Math.abs(Math.cos(c.stride / 115)) * 12 : 0);
    const S = c.shoulder, H = c.hand;
    const L1 = 300, L2 = 270;
    const toH = sub(H, S);
    let d = norm(toH);
    const u = normalize(toH);
    d = Math.max(60, Math.min(d, L1 + L2 - 1));
    // codo hacia abajo y hacia afuera (lado derecho del cliente)
    const out: Vec3 = [0.6 * Math.cos(c.yaw), 0.6 * Math.sin(c.yaw), -1];
    const pole = normalize(sub(out, scale(u, dot(out, u))));
    const a = (L1 * L1 - L2 * L2 + d * d) / (2 * d);
    const h = Math.sqrt(Math.max(0, L1 * L1 - a * a));
    const E = add(add(S, scale(u, a)), scale(pole, h));
    const Hc = add(S, scale(u, d));
    placeCapsule(cv.upper, S, E);
    placeCapsule(cv.fore, E, Hc);
    cv.hand.position.set(Hc[0], Hc[1], Hc[2]);
    const dir = normalize(sub(Hc, E));
    cv.hand.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3(...dir));
    cv.fingers.scale.set(1, c.handClosed ? 0.55 : 1, c.handClosed ? 1.6 : 1);
    cv.fingers.position.set(0, c.handClosed ? 45 : 60, c.handClosed ? 10 : 0);
  }

  private syncButton(sim: Simulation) {
    if (!this.button) return;
    const on = !!sim.signals.get('PEDIDO');
    const t = performance.now() / 1000;
    this.button.mat.emissiveIntensity = on ? 0.9 + 0.5 * Math.sin(t * 6) : 0.05;
    this.button.ring.emissiveIntensity = on ? 1.2 : 0.0;
    // el hongo se hunde mientras la mano lo presiona
    const c = sim.sc.customer;
    let press = 0;
    if (c && c.present) {
      const h = c.hand;
      const dxy = Math.hypot(h[0] - BUTTON.base[0], h[1] - BUTTON.base[1]);
      if (dxy < 60) press = Math.max(0, Math.min(1, (BUTTON.top + 70 - h[2]) / 30));
    }
    this.button.cap.position.z = BUTTON.top + 4 - press * 10;
  }

  private syncEnvelopes(sim: Simulation) {
    this.envelope.visible = this.showEnvelopes;
    if (!this.showEnvelopes) return;
    const all: [number, number, number, number, string][] = [];
    for (const r of sim.sc.robots) for (const l of robotSpheres(r)) for (const s of l.s) all.push([s[0], s[1], s[2], s[3], r.id === 'ur' ? '#22c3d0' : '#e2b400']);
    let mesh = this.envelope.children[0] as THREE.InstancedMesh | undefined;
    if (!mesh || mesh.count < all.length) {
      this.envelope.clear();
      mesh = new THREE.InstancedMesh(new THREE.SphereGeometry(1, 12, 8), new THREE.MeshBasicMaterial({ transparent: true, opacity: 0.16, depthWrite: false }), Math.max(all.length, 600));
      this.envelope.add(mesh);
    }
    const mtx = new THREE.Matrix4(), col = new THREE.Color();
    all.forEach((s, i) => {
      mtx.makeScale(s[3], s[3], s[3]).setPosition(s[0], s[1], s[2]);
      mesh!.setMatrixAt(i, mtx);
      mesh!.setColorAt(i, col.set(s[4]));
    });
    mesh.count = all.length;
    mesh.instanceMatrix.needsUpdate = true;
    if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
  }

  private syncLamp(sim: Simulation) {
    if (!this.lamp) return;
    const waiting = sim.sc.robots.some((r) => r.state === 'esperando');
    const err = sim.errors > 0;
    const moving = sim.sc.robots.some((r) => r.state === 'moviendo' || r.state === 'lineal' || r.state === 'pinza');
    this.lamp.red.emissiveIntensity = err ? 2.2 : 0.05;
    this.lamp.amber.emissiveIntensity = !err && waiting ? 1.8 : 0.05;
    this.lamp.green.emissiveIntensity = !err && moving ? 1.8 : 0.05;
  }
}

/**
 * BoxGeometry asume Y hacia arriba: en nuestras caras ±Y (verticales, porque el mundo es Z-arriba) el texto quedaría
 * de costado o espejado. Se recalculan las UV para que se lea derecho mirando cada cara desde afuera.
 */
function fixVerticalFaceUV(geo: THREE.BoxGeometry) {
  const pos = geo.getAttribute('position') as THREE.BufferAttribute;
  const uv = geo.getAttribute('uv') as THREE.BufferAttribute;
  geo.computeBoundingBox();
  const bb = geo.boundingBox!;
  for (let i = 8; i < 16; i++) {
    const x = (pos.getX(i) - bb.min.x) / (bb.max.x - bb.min.x);
    const z = (pos.getZ(i) - bb.min.z) / (bb.max.z - bb.min.z);
    // cara +Y (8-11) se mira desde +Y: la X del mundo va hacia la izquierda; cara -Y (12-15): hacia la derecha
    uv.setXY(i, i < 12 ? 1 - x : x, z);
  }
  uv.needsUpdate = true;
}

function placeCapsule(m: THREE.Mesh, a: Vec3, b: Vec3) {
  const mid = scale(add(a, b), 0.5);
  m.position.set(mid[0], mid[1], mid[2]);
  const dir = normalize(sub(b, a));
  m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3(dir[0], dir[1], dir[2]));
}
