/**
 * Cinemática de los cobots con producto de exponenciales (POE).
 * Cada articulación es un eje (punto + dirección) medido sobre la malla original en la pose de exportación (q = 0).
 * La geometría de cada eslabón está en ese mismo marco, así que su pose es simplemente E1·…·Ek.
 */
import {
  type Mat3, type Pose, type Vec3, I3, add, axisAngle, cross, fromCols, mul3, mv, normalize, pose, poseInv, poseMul,
  rotLog, scale, solve, sub, rotZ, norm,
} from './math';

export interface JointDef { point: Vec3; axis: Vec3 }
export interface RobotData {
  name: string;
  joints: JointDef[];
  flange: { point: Vec3; z: Vec3; x: Vec3 };
  limits: [number, number][];
  links: { parts: { name: string; color: string; positions: string; indices: string; index32: boolean }[]; spheres: number[][] }[];
}

export interface IkResult { q: number[]; ok: boolean; posErr: number; rotErr: number; iters: number }

export class RobotKin {
  readonly n = 6;
  readonly joints: JointDef[];
  readonly limits: [number, number][];
  /** Brida en q = 0 (marco base del robot). */
  readonly M: Pose;
  base: Pose;
  tool: Pose = pose([0, 0, 0]);

  constructor(readonly data: RobotData, basePos: Vec3, baseYaw: number) {
    this.joints = data.joints.map((j) => ({ point: j.point, axis: normalize(j.axis) }));
    this.limits = data.limits;
    const z = normalize(data.flange.z), x = normalize(data.flange.x);
    this.M = pose(data.flange.point, fromCols(x, cross(z, x), z));
    this.base = pose(basePos, rotZ(baseYaw));
  }

  private expJoint(i: number, q: number): Pose {
    const j = this.joints[i];
    const R = axisAngle(j.axis, q);
    return { R, p: sub(j.point, mv(R, j.point)) };
  }

  /** Poses de los 7 eslabones (0 = base fija) en el mundo, para mover la geometría capturada en q = 0. */
  linkPoses(q: number[]): Pose[] {
    const out: Pose[] = [this.base];
    let G: Pose = this.base;
    for (let i = 0; i < 6; i++) {
      G = poseMul(G, this.expJoint(i, q[i]));
      out.push(G);
    }
    return out;
  }

  flangePose(q: number[]): Pose {
    return poseMul(this.linkPoses(q)[6], this.M);
  }

  tcpPose(q: number[]): Pose {
    return poseMul(this.flangePose(q), this.tool);
  }

  /** Jacobiano geométrico 6×6 (filas: vx vy vz wx wy wz) del TCP en el mundo. */
  jacobian(q: number[]): { J: number[]; tcp: Pose } {
    const L = this.linkPoses(q);
    const tcp = poseMul(poseMul(L[6], this.M), this.tool);
    const J = new Array<number>(36).fill(0);
    for (let i = 0; i < 6; i++) {
      const G = L[i];
      const w = mv(G.R, this.joints[i].axis);
      const pt = add(mv(G.R, this.joints[i].point), G.p);
      const v = cross(w, sub(tcp.p, pt));
      J[0 * 6 + i] = v[0]; J[1 * 6 + i] = v[1]; J[2 * 6 + i] = v[2];
      J[3 * 6 + i] = w[0]; J[4 * 6 + i] = w[1]; J[5 * 6 + i] = w[2];
    }
    return { J, tcp };
  }

  /**
   * IK numérica por mínimos cuadrados amortiguados, partiendo de `seed` (normalmente la posición actual):
   * así la solución es continua con la configuración previa y no hay saltos de codo o muñeca.
   */
  ik(target: Pose, seed: number[], opts: { maxIter?: number; tolPos?: number; tolRot?: number } = {}): IkResult {
    const maxIter = opts.maxIter ?? 120, tolPos = opts.tolPos ?? 0.05, tolRot = opts.tolRot ?? 0.0005;
    const W = 250; // mm por rad: pondera el error de orientación frente al de posición
    const q = seed.slice();
    let posErr = Infinity, rotErr = Infinity, it = 0;
    for (; it < maxIter; it++) {
      const { J, tcp } = this.jacobian(q);
      const ep = sub(target.p, tcp.p);
      const eo = mv(tcp.R, rotLog(mul3(poseInv(tcp).R, target.R)));
      posErr = norm(ep); rotErr = norm(eo);
      if (posErr < tolPos && rotErr < tolRot) break;
      // limitar el paso cartesiano para que la iteración sea estable lejos de la solución
      const kp = Math.min(1, 60 / (posErr || 1)), ko = Math.min(1, 0.35 / (rotErr || 1));
      const e = [ep[0] * kp, ep[1] * kp, ep[2] * kp, eo[0] * ko * W, eo[1] * ko * W, eo[2] * ko * W];
      const Jw = J.slice();
      for (let c = 0; c < 6; c++) for (let r = 3; r < 6; r++) Jw[r * 6 + c] *= W;
      // (J Jᵀ + λ² I) y = e ;  dq = Jᵀ y
      const lambda = 2.0;
      const A = new Array<number>(36).fill(0);
      for (let r = 0; r < 6; r++)
        for (let c = 0; c < 6; c++) {
          let s = 0;
          for (let k = 0; k < 6; k++) s += Jw[r * 6 + k] * Jw[c * 6 + k];
          A[r * 6 + c] = s + (r === c ? lambda * lambda : 0);
        }
      const y = solve(A, e, 6);
      for (let k = 0; k < 6; k++) {
        let d = 0;
        for (let r = 0; r < 6; r++) d += Jw[r * 6 + k] * y[r];
        q[k] = Math.min(this.limits[k][1], Math.max(this.limits[k][0], q[k] + d));
      }
    }
    return { q, ok: posErr < 0.5 && rotErr < 0.005, posErr, rotErr, iters: it };
  }
}

/**
 * IK robusta para movimientos articulares: primero desde la posición actual; si no converge, prueba una grilla fija de
 * semillas y se queda con la solución válida MÁS CERCANA a la posición actual (movimiento mínimo, sin vueltas raras).
 */
export function ikNear(k: RobotKin, T: Pose, current: number[]): IkResult {
  const first = k.ik(T, current, { maxIter: 300 });
  if (first.ok) return first;
  let best: IkResult | null = null, bestD = Infinity;
  const v = [-2.4, -1.2, 0, 1.2, 2.4];
  for (const a of v) for (const b of [-0.8, 0, 0.8]) for (const c of [-1.6, 0, 1.6]) for (const d of [-1.2, 0, 1.2]) for (const e of [-1, 1]) {
    const r = k.ik(T, [a, b, c, d, e, current[5]], { maxIter: 250 });
    if (!r.ok) continue;
    // refinar desde la solución para que la última articulación quede lo más cerca posible de la actual
    for (const i of [0, 3, 5]) {
      const tw = 2 * Math.PI;
      while (r.q[i] - current[i] > Math.PI && r.q[i] - tw >= k.limits[i][0]) r.q[i] -= tw;
      while (current[i] - r.q[i] > Math.PI && r.q[i] + tw <= k.limits[i][1]) r.q[i] += tw;
    }
    const dq = r.q.reduce((s, x, i) => s + Math.abs(x - current[i]), 0);
    if (dq < bestD) { bestD = dq; best = r; }
  }
  return best ?? first;
}

/** Pose objetivo del TCP: posición + matriz de orientación. */
export const target = (p: Vec3, R: Mat3 = I3()): Pose => ({ p, R });
export const offsetAlongZ = (P: Pose, d: number): Pose => ({ R: P.R, p: add(P.p, scale([P.R[2], P.R[5], P.R[8]], d)) });
