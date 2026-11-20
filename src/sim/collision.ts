/**
 * Verificación de colisiones durante la simulación.
 *  - Robot contra robot: esferas envolventes calculadas sobre la malla original de cada eslabón + pinza + objeto sujeto.
 *  - Dedos de las pinzas contra mesa, soportes y objetos no sujetos: cajas orientadas (test de ejes separadores).
 *  - Objeto transportado contra los demás objetos y la mesa.
 * Los contactos previstos (apoyar un cubo sobre otro) se toleran con un margen de 0,5 mm.
 */
import { type Mat3, type Vec3, add, col, dist, poseApply, poseMul, I3 } from '../kin/math';
import { type RobotActor, type Simulation, type SimObject } from './engine';
import { GRIPPER, fingerBoxes, gripperSpheres } from './gripper';

interface OBB { c: Vec3; R: Mat3; h: Vec3; name: string }
type Sphere = [number, number, number, number];

const TOL = 0.5;

function obbOverlap(a: OBB, b: OBB): boolean {
  // Teorema de ejes separadores para dos cajas orientadas (15 ejes)
  const A = [col(a.R, 0), col(a.R, 1), col(a.R, 2)];
  const B = [col(b.R, 0), col(b.R, 1), col(b.R, 2)];
  const T: Vec3 = [b.c[0] - a.c[0], b.c[1] - a.c[1], b.c[2] - a.c[2]];
  const ha = a.h.map((v) => Math.max(0, v - TOL / 2)), hb = b.h.map((v) => Math.max(0, v - TOL / 2));
  const axes: Vec3[] = [...A, ...B];
  for (const u of A) for (const v of B) {
    const c: Vec3 = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    const n = Math.hypot(c[0], c[1], c[2]);
    if (n > 1e-6) axes.push([c[0] / n, c[1] / n, c[2] / n]);
  }
  const d = (x: Vec3, y: Vec3) => x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
  for (const L of axes) {
    const ra = ha[0] * Math.abs(d(A[0], L)) + ha[1] * Math.abs(d(A[1], L)) + ha[2] * Math.abs(d(A[2], L));
    const rb = hb[0] * Math.abs(d(B[0], L)) + hb[1] * Math.abs(d(B[1], L)) + hb[2] * Math.abs(d(B[2], L));
    if (Math.abs(d(T, L)) > ra + rb) return false;
  }
  return true;
}

function objectBox(o: SimObject): OBB {
  return { c: o.pose.p, R: o.pose.R, h: o.half, name: o.label };
}

export function robotSpheres(r: RobotActor): { link: number; s: Sphere[] }[] {
  const L = r.kin.linkPoses(r.q);
  const out: { link: number; s: Sphere[] }[] = [];
  for (let i = 1; i <= 6; i++) {
    const P = L[i];
    out.push({ link: i, s: r.kin.data.links[i].spheres.map((s) => { const c = poseApply(P, [s[0], s[1], s[2]]); return [c[0], c[1], c[2], s[3]] as Sphere; }) });
  }
  const F = poseMul(L[6], r.kin.M);
  out.push({ link: 7, s: gripperSpheres(r.gripWidth).map((s) => { const c = poseApply(F, [s[0], s[1], s[2]]); return [c[0], c[1], c[2], s[3]] as Sphere; }) });
  return out;
}

function bound(s: Sphere[]): Sphere {
  let c: Vec3 = [0, 0, 0];
  for (const x of s) c = add(c, [x[0] / s.length, x[1] / s.length, x[2] / s.length]);
  let r = 0;
  for (const x of s) r = Math.max(r, dist(c, [x[0], x[1], x[2]]) + x[3]);
  return [c[0], c[1], c[2], r];
}

function minDist(a: Sphere[], b: Sphere[]): number {
  let m = Infinity;
  for (const x of a) for (const y of b) {
    const d = Math.hypot(x[0] - y[0], x[1] - y[1], x[2] - y[2]) - x[3] - y[3];
    if (d < m) m = d;
  }
  return m;
}

export function fingerOBBs(r: RobotActor): OBB[] {
  const F = r.kin.flangePose(r.q);
  return fingerBoxes(r.gripWidth).map((b, i) => ({ c: poseApply(F, b.c), R: F.R, h: b.h, name: `dedo ${i + 1} de ${r.label}` }));
}

export function gripperBodyOBB(r: RobotActor): OBB {
  const F = r.kin.flangePose(r.q);
  const b = GRIPPER.body;
  return { c: poseApply(F, [0, 0, (b.z0 + b.z1) / 2]), R: F.R, h: [b.hx, b.hy, (b.z1 - b.z0) / 2], name: `cuerpo de pinza de ${r.label}` };
}

// vértices reales de cada eslabón (muestreados), para confirmar autocolisiones
const vertCache = new Map<string, Vec3[]>();
function linkVerts(r: RobotActor, link: number): Vec3[] {
  const key = `${r.kin.data.name}|${link}`;
  let v = vertCache.get(key);
  if (!v) {
    v = [];
    if (link === 7) {
      // pinza: esquinas y puntos medios de cuerpo y dedos (marco de la brida)
      const b = GRIPPER.body;
      for (const x of [-b.hx, 0, b.hx]) for (const y of [-b.hy, 0, b.hy]) for (const z of [b.z0, b.z1]) v.push([x, y, z]);
    } else {
      for (const p of r.kin.data.links[link].parts) {
        const bin = atob(p.positions);
        const buf = new ArrayBuffer(bin.length), u8 = new Uint8Array(buf);
        for (let i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
        const f = new Float32Array(buf);
        for (let i = 0; i < f.length; i += 6) v.push([f[i], f[i + 1], f[i + 2]]);
      }
    }
    vertCache.set(key, v);
  }
  return v;
}
function meshDistance(r: RobotActor, la: number, lb: number): number {
  const L = r.kin.linkPoses(r.q);
  const F = poseMul(L[6], r.kin.M);
  const A = linkVerts(r, la).map((p) => poseApply(la === 7 ? F : L[la], p));
  const B = linkVerts(r, lb).map((p) => poseApply(L[lb], p));
  let m = Infinity;
  for (const x of A) for (const y of B) {
    const d = (x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2 + (x[2] - y[2]) ** 2;
    if (d < m) m = d;
  }
  return Math.sqrt(m);
}

/** Instala la verificación en la simulación. Registra cada contacto indebido una sola vez por par. */
export function installCollision(sim: Simulation) {
  const reported = new Set<string>();
  const lastMesh = new Map<string, number>();
  const report = (key: string, text: string) => {
    if (reported.has(key)) return;
    reported.add(key);
    sim.emit('Colisiones', text, 'error');
  };
  const statics: OBB[] = sim.sc.statics.map((s) => ({ c: s.c, R: I3(), h: s.h, name: s.id }));

  sim.collisionHook = () => {
    const robots = sim.sc.robots;
    // 1) robot contra robot (con esferas envolventes, conservador)
    if (robots.length === 2) {
      const A = robotSpheres(robots[0]), B = robotSpheres(robots[1]);
      let m = Infinity;
      for (const a of A) {
        const ba = bound(a.s);
        for (const b of B) {
          const bb = bound(b.s);
          if (Math.hypot(ba[0] - bb[0], ba[1] - bb[1], ba[2] - bb[2]) - ba[3] - bb[3] > m) continue;
          const d = minDist(a.s, b.s);
          if (d < m) m = d;
        }
      }
      sim.clearNow = m;
      sim.minClear = Math.min(sim.minClear, m);
      if (m < 0) report('rr', `Robots demasiado cerca: interferencia de ${(-m).toFixed(0)} mm entre UR5 y Turin`);
    }
    // 2) eslabones del brazo (2 a 6) contra la mesa: ninguna esfera envolvente puede bajar del tablero
    const top = statics.length ? statics[0].c[2] + statics[0].h[2] : 0;
    for (const r of robots) {
      const S = robotSpheres(r);
      for (const l of S)
        if (l.link >= 2 && l.link <= 6)
          for (const s of l.s) {
            if (s[2] - s[3] < top - 2) report(`${r.id}|l${l.link}|mesa`, `Contacto: eslabón ${l.link} de ${r.label} con la mesa`);
            // y contra soportes, bandejas y base de la torre (cajas fijas sobre la mesa)
            for (const b of statics.slice(1)) {
              const dx = Math.max(0, Math.abs(s[0] - b.c[0]) - b.h[0]);
              const dy = Math.max(0, Math.abs(s[1] - b.c[1]) - b.h[1]);
              const dz = Math.max(0, Math.abs(s[2] - b.c[2]) - b.h[2]);
              if (Math.hypot(dx, dy, dz) < s[3]) report(`${r.id}|l${l.link}|${b.name}`, `Contacto: eslabón ${l.link} de ${r.label} con ${b.name}`);
            }
          }
      // 2b) autocolisión: muñeca y pinza (eslabones 5, 6 y pinza) contra hombro y brazo (1 y 2), pares no contiguos.
      // Las envolventes son conservadoras: si se tocan, se confirma con los vértices reales de la malla.
      for (const a of S.filter((x) => x.link >= 5))
        for (const b of S.filter((x) => x.link <= 2)) {
          if (minDist(a.s, b.s) >= 0) continue;
          const key = `${r.id}|self|${a.link}|${b.link}`;
          if (reported.has(key) || (lastMesh.get(key) ?? -1) > sim.time - 0.2) continue;
          lastMesh.set(key, sim.time);
          const d = meshDistance(r, a.link, b.link);
          if (d < 2) report(key, `Autocolisión en ${r.label}: ${a.link === 7 ? 'pinza' : `eslabón ${a.link}`} contra eslabón ${b.link}`);
        }
    }
    // 3) dedos y cuerpo de pinza contra mesa/soportes y contra objetos que no sujeta
    for (const r of robots) {
      const parts = [...fingerOBBs(r), gripperBodyOBB(r)];
      for (const f of parts) {
        for (const s of statics) if (obbOverlap(f, s)) report(`${f.name}|${s.name}`, `Contacto: ${f.name} con ${s.name}`);
        for (const o of sim.sc.objects) {
          if (o.holders.length) continue; // los objetos sujetos los agarran justamente los dedos
          if (obbOverlap(f, objectBox(o))) report(`${f.name}|${o.id}`, `Contacto: ${f.name} con ${o.label}`);
        }
      }
    }
    // 4) objeto transportado contra los demás objetos y la mesa
    for (const o of sim.sc.objects) {
      if (!o.holders.length || o.kind !== 'cube') continue;
      const ob = objectBox(o);
      for (const s of statics) if (obbOverlap(ob, s)) report(`${o.id}|${s.name}`, `Contacto: ${o.label} atraviesa ${s.name}`);
      for (const p of sim.sc.objects) {
        if (p === o || p.holders.length) continue;
        if (obbOverlap(ob, objectBox(p))) report(`${o.id}|${p.id}`, `Contacto: ${o.label} con ${p.label}`);
      }
      // y contra la pinza del otro robot
      for (const r of robots) {
        if (o.holders.some((h) => h.by === r.id)) continue;
        for (const f of [...fingerOBBs(r), gripperBodyOBB(r)])
          if (obbOverlap(ob, f)) report(`${o.id}|${f.name}`, `Contacto: ${o.label} con ${f.name}`);
      }
    }
  };
}
