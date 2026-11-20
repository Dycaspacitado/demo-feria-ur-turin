import robotsJson from '../models/robots.json';
import { type Mat3, type Pose, type Vec3, add, frameZY, pose, rotZ, mv } from '../kin/math';
import { RobotKin, type RobotData } from '../kin/robot';
import { GRIPPER } from './gripper';
import { type Step } from './engine';

export const ROBOTS = robotsJson as unknown as Record<'UR5' | 'TCR050', RobotData>;

/** Mesa de trabajo común: tablero de 1800 × 1000 mm, superficie en z = 0 (altura 750 mm sobre el piso). */
export const TABLE = { x0: -900, x1: 900, y0: -500, y1: 500, top: 0, thick: 30, floor: -750 };
export const tableStatic = { id: 'la mesa', c: [0, 0, -TABLE.thick / 2] as Vec3, h: [900, 500, TABLE.thick / 2] as Vec3 };

export function makeKin(model: 'UR5' | 'TCR050', base: Vec3, yawDeg: number): RobotKin {
  const k = new RobotKin(ROBOTS[model], base, (yawDeg * Math.PI) / 180);
  k.tool = pose([0, 0, GRIPPER.tcpZ]);
  return k;
}

/** Orientaciones de herramienta usadas (Z = dirección de aproximación, Y = cierre de mordazas). */
export const DOWN_JAW_Y: Mat3 = frameZY([0, 0, -1], [0, 1, 0]);
export const DOWN_JAW_X: Mat3 = frameZY([0, 0, -1], [1, 0, 0]);

/**
 * Resuelve la pose de reposo (Home) eligiendo, entre varias semillas fijas (resultado determinista), la postura
 * más "ordenada": codo arriba, muñeca lejos de la singularidad (ejes 4 y 6 no paralelos) y giros articulares pequeños.
 * El resto del programa se resuelve siempre partiendo de la posición actual, así que la postura se conserva.
 */
export function solveHome(k: RobotKin, tcp: Pose): number[] {
  let best: { q: number[]; cost: number } | null = null;
  const vals = [-2.4, -1.2, 0, 1.2, 2.4];
  for (const a of vals) for (const b of [-0.8, 0, 0.8]) for (const c of [-1.6, 0, 1.6]) for (const d of [-1.2, 0, 1.2]) {
    const r = k.ik(tcp, [a, b, c, d, 1.0, 0], { maxIter: 250 });
    if (!r.ok) continue;
    const L = k.linkPoses(r.q);
    const elbow = L[2].R[6] * k.joints[2].point[0] + L[2].R[7] * k.joints[2].point[1] + L[2].R[8] * k.joints[2].point[2] + L[2].p[2];
    const a4 = mv(L[3].R, k.joints[3].axis), a6 = mv(L[5].R, k.joints[5].axis);
    const sinW = Math.hypot(a4[1] * a6[2] - a4[2] * a6[1], a4[2] * a6[0] - a4[0] * a6[2], a4[0] * a6[1] - a4[1] * a6[0]);
    // la muñeca (eje 4) debe quedar del lado del propio robot, no más allá del TCP: así el antebrazo
    // no invade el espacio del otro robot cuando trabaja en la zona compartida
    const w1 = add(mv(L[3].R, k.joints[3].point), L[3].p);
    const hd = (p: Vec3) => Math.hypot(p[0] - k.base.p[0], p[1] - k.base.p[1]);
    const overreach = Math.max(0, hd(w1) - hd(tcp.p));
    const cost = -elbow / 150 - 3 * sinW + 0.25 * r.q.reduce((s, x) => s + Math.abs(x), 0) + overreach / 25;
    if (!best || cost < best.cost) best = { q: r.q, cost };
  }
  if (!best) throw new Error(`Home inalcanzable para ${k.data.name}`);
  return best.q;
}

/** Pose relativa a la base del robot (útil para definir el Home en coordenadas del robot). */
export function inBase(k: RobotKin, p: Vec3, R: Mat3): Pose {
  return { p: [k.base.p[0] + mv(k.base.R, p)[0], k.base.p[1] + mv(k.base.R, p)[1], k.base.p[2] + mv(k.base.R, p)[2]], R };
}

export const above = (P: Pose, dz: number): Pose => ({ R: P.R, p: [P.p[0], P.p[1], P.p[2] + dz] });
export const shift = (P: Pose, d: Vec3): Pose => ({ R: P.R, p: [P.p[0] + d[0], P.p[1] + d[1], P.p[2] + d[2]] });
export const rotYaw = (R: Mat3, deg: number): Mat3 => {
  const Z = rotZ((deg * Math.PI) / 180);
  return [
    Z[0] * R[0] + Z[1] * R[3] + Z[2] * R[6], Z[0] * R[1] + Z[1] * R[4] + Z[2] * R[7], Z[0] * R[2] + Z[1] * R[5] + Z[2] * R[8],
    Z[3] * R[0] + Z[4] * R[3] + Z[5] * R[6], Z[3] * R[1] + Z[4] * R[4] + Z[5] * R[7], Z[3] * R[2] + Z[4] * R[5] + Z[5] * R[8],
    Z[6] * R[0] + Z[7] * R[3] + Z[8] * R[6], Z[6] * R[1] + Z[7] * R[4] + Z[8] * R[7], Z[6] * R[2] + Z[7] * R[5] + Z[8] * R[8],
  ];
};

// --------------------------------------------------------------------- atajos para escribir programas
export const op = (text: string): Step => ({ k: 'op', text });
export const movej = (to: Pose | number[], text?: string, speed?: number): Step => ({ k: 'movej', to, text, speed });
export const movel = (to: Pose, text?: string, speed?: number): Step => ({ k: 'movel', to, text, speed });
export const grip = (width: number, obj?: string, text?: string): Step => ({ k: 'grip', width, obj, text });
export const wait = (t: number, text?: string): Step => ({ k: 'wait', t, text });
export const set = (sig: string, value = true): Step => ({ k: 'set', sig, value });
export const waitsig = (sig: string, text?: string, value = true): Step => ({ k: 'waitsig', sig, value, text });
