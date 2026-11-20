/**
 * Álgebra mínima sin dependencias (testeable en Node). Unidades: mm y rad.
 * Mat3: 9 números fila-mayor. Pose: rotación R (Mat3) + posición p.
 */
export type Vec3 = [number, number, number];
export type Mat3 = number[];
export interface Pose { R: Mat3; p: Vec3 }

export const v3 = (x: number, y: number, z: number): Vec3 => [x, y, z];
export const add = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const scale = (a: Vec3, k: number): Vec3 => [a[0] * k, a[1] * k, a[2] * k];
export const dot = (a: Vec3, b: Vec3) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross = (a: Vec3, b: Vec3): Vec3 => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
export const norm = (a: Vec3) => Math.hypot(a[0], a[1], a[2]);
export const normalize = (a: Vec3): Vec3 => scale(a, 1 / (norm(a) || 1));
export const lerp3 = (a: Vec3, b: Vec3, t: number): Vec3 => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
export const dist = (a: Vec3, b: Vec3) => norm(sub(a, b));

export const I3 = (): Mat3 => [1, 0, 0, 0, 1, 0, 0, 0, 1];
export function mul3(a: Mat3, b: Mat3): Mat3 {
  const o = new Array<number>(9);
  for (let r = 0; r < 3; r++)
    for (let c = 0; c < 3; c++) o[r * 3 + c] = a[r * 3] * b[c] + a[r * 3 + 1] * b[3 + c] + a[r * 3 + 2] * b[6 + c];
  return o;
}
export const T3 = (a: Mat3): Mat3 => [a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]];
export const mv = (m: Mat3, v: Vec3): Vec3 => [
  m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
  m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
  m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
];
export const col = (m: Mat3, c: number): Vec3 => [m[c], m[3 + c], m[6 + c]];
/** Matriz a partir de sus columnas X, Y, Z. */
export const fromCols = (x: Vec3, y: Vec3, z: Vec3): Mat3 => [x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]];

/** Rotación de ángulo t alrededor del eje unitario w (Rodrigues). */
export function axisAngle(w: Vec3, t: number): Mat3 {
  const c = Math.cos(t), s = Math.sin(t), k = 1 - c;
  const [x, y, z] = w;
  return [
    c + x * x * k, x * y * k - z * s, x * z * k + y * s,
    y * x * k + z * s, c + y * y * k, y * z * k - x * s,
    z * x * k - y * s, z * y * k + x * s, c + z * z * k,
  ];
}
export const rotZ = (t: number) => axisAngle([0, 0, 1], t);
export const rotX = (t: number) => axisAngle([1, 0, 0], t);
export const rotY = (t: number) => axisAngle([0, 1, 0], t);

/** Vector de rotación (eje·ángulo) de una matriz de rotación: log(R). */
export function rotLog(R: Mat3): Vec3 {
  const tr = R[0] + R[4] + R[8];
  const c = Math.min(1, Math.max(-1, (tr - 1) / 2));
  const t = Math.acos(c);
  if (t < 1e-9) return [0, 0, 0];
  if (Math.PI - t < 1e-6) {
    // ~180°: eje desde la diagonal
    const x = Math.sqrt(Math.max(0, (R[0] + 1) / 2)), y = Math.sqrt(Math.max(0, (R[4] + 1) / 2)), z = Math.sqrt(Math.max(0, (R[8] + 1) / 2));
    const ax: Vec3 = [x, R[1] >= 0 ? y : -y, R[2] >= 0 ? z : -z];
    return scale(normalize(ax), t);
  }
  const k = t / (2 * Math.sin(t));
  return [(R[7] - R[5]) * k, (R[2] - R[6]) * k, (R[3] - R[1]) * k];
}

export const poseMul = (a: Pose, b: Pose): Pose => ({ R: mul3(a.R, b.R), p: add(mv(a.R, b.p), a.p) });
export const poseInv = (a: Pose): Pose => {
  const Rt = T3(a.R);
  return { R: Rt, p: scale(mv(Rt, a.p), -1) };
};
export const poseApply = (a: Pose, v: Vec3): Vec3 => add(mv(a.R, v), a.p);
export const pose = (p: Vec3, R: Mat3 = I3()): Pose => ({ R, p });

// ---- cuaterniones (para interpolar orientaciones en movimientos lineales)
export type Quat = [number, number, number, number]; // x y z w
export function matToQuat(m: Mat3): Quat {
  const tr = m[0] + m[4] + m[8];
  let x: number, y: number, z: number, w: number;
  if (tr > 0) {
    const s = Math.sqrt(tr + 1) * 2;
    w = 0.25 * s; x = (m[7] - m[5]) / s; y = (m[2] - m[6]) / s; z = (m[3] - m[1]) / s;
  } else if (m[0] > m[4] && m[0] > m[8]) {
    const s = Math.sqrt(1 + m[0] - m[4] - m[8]) * 2;
    w = (m[7] - m[5]) / s; x = 0.25 * s; y = (m[1] + m[3]) / s; z = (m[2] + m[6]) / s;
  } else if (m[4] > m[8]) {
    const s = Math.sqrt(1 + m[4] - m[0] - m[8]) * 2;
    w = (m[2] - m[6]) / s; x = (m[1] + m[3]) / s; y = 0.25 * s; z = (m[5] + m[7]) / s;
  } else {
    const s = Math.sqrt(1 + m[8] - m[0] - m[4]) * 2;
    w = (m[3] - m[1]) / s; x = (m[2] + m[6]) / s; y = (m[5] + m[7]) / s; z = 0.25 * s;
  }
  const n = Math.hypot(x, y, z, w);
  return [x / n, y / n, z / n, w / n];
}
export function quatToMat(q: Quat): Mat3 {
  const [x, y, z, w] = q;
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
  ];
}
export function slerp(a: Quat, b: Quat, t: number): Quat {
  let d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  let bb = b;
  if (d < 0) { d = -d; bb = [-b[0], -b[1], -b[2], -b[3]]; }
  if (d > 0.9995) {
    const r: Quat = [a[0] + (bb[0] - a[0]) * t, a[1] + (bb[1] - a[1]) * t, a[2] + (bb[2] - a[2]) * t, a[3] + (bb[3] - a[3]) * t];
    const n = Math.hypot(...r);
    return [r[0] / n, r[1] / n, r[2] / n, r[3] / n];
  }
  const th = Math.acos(d), s = Math.sin(th);
  const ka = Math.sin((1 - t) * th) / s, kb = Math.sin(t * th) / s;
  return [a[0] * ka + bb[0] * kb, a[1] * ka + bb[1] * kb, a[2] * ka + bb[2] * kb, a[3] * ka + bb[3] * kb];
}
export const rotAngle = (a: Mat3, b: Mat3) => norm(rotLog(mul3(T3(a), b)));

/** Herramienta "apuntando" en dirección z con las mordazas cerrando a lo largo de y. */
export function frameZY(z: Vec3, y: Vec3): Mat3 {
  const zz = normalize(z);
  const yy = normalize(sub(y, scale(zz, dot(y, zz))));
  return fromCols(cross(yy, zz), yy, zz);
}

/** Perfil de mínimo tirón (quíntico): posición, con velocidad y aceleración nulas en los extremos. */
export const minJerk = (t: number) => {
  const s = Math.min(1, Math.max(0, t));
  return s * s * s * (10 + s * (-15 + 6 * s));
};
/** Pico de velocidad y aceleración normalizadas del perfil quíntico. */
export const MINJERK_VPEAK = 1.875;
export const MINJERK_APEAK = 5.7735;

/** Resuelve A x = b (n×n, fila-mayor) por eliminación gaussiana con pivoteo parcial. */
export function solve(A: number[], b: number[], n: number): number[] {
  const M = A.slice(), x = b.slice();
  for (let c = 0; c < n; c++) {
    let piv = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r * n + c]) > Math.abs(M[piv * n + c])) piv = r;
    if (piv !== c) {
      for (let k = 0; k < n; k++) { const t = M[c * n + k]; M[c * n + k] = M[piv * n + k]; M[piv * n + k] = t; }
      const t = x[c]; x[c] = x[piv]; x[piv] = t;
    }
    const d = M[c * n + c] || 1e-12;
    for (let r = c + 1; r < n; r++) {
      const f = M[r * n + c] / d;
      if (!f) continue;
      for (let k = c; k < n; k++) M[r * n + k] -= f * M[c * n + k];
      x[r] -= f * x[c];
    }
  }
  for (let r = n - 1; r >= 0; r--) {
    let s = x[r];
    for (let k = r + 1; k < n; k++) s -= M[r * n + k] * x[k];
    x[r] = s / (M[r * n + r] || 1e-12);
  }
  return x;
}
