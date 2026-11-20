/**
 * Pinza paralela de dos dedos (modelo propio, independiente de los robots). Cotas en mm, en el marco de la brida:
 * Z sale de la brida hacia los dedos, Y es la dirección de cierre de las mordazas.
 * Las mismas cotas usan la vista 3D y la verificación de colisiones.
 */
export const GRIPPER = {
  adapter: { r: 33, z0: 0, z1: 10 },
  body: { hx: 28, hy: 50, z0: 10, z1: 72 },
  finger: { hx: 10, t: 10, z0: 72, z1: 120 },
  /** El TCP queda 12 mm por encima de la punta de los dedos: centro de las almohadillas de agarre. */
  tcpZ: 108,
  maxOpen: 86,
  /** Velocidad de apertura/cierre (mm/s) */
  speed: 110,
};

/** Cajas (centro y semiejes, marco de la brida) de los dos dedos para una apertura dada. */
export function fingerBoxes(width: number) {
  const f = GRIPPER.finger;
  const cz = (f.z0 + f.z1) / 2, hz = (f.z1 - f.z0) / 2;
  const cy = width / 2 + f.t / 2;
  return [
    { c: [0, cy, cz] as [number, number, number], h: [f.hx, f.t / 2, hz] as [number, number, number] },
    { c: [0, -cy, cz] as [number, number, number], h: [f.hx, f.t / 2, hz] as [number, number, number] },
  ];
}

/** Esferas envolventes de cuerpo y adaptador de la pinza (marco de la brida): para robot contra robot. */
export function gripperSpheres(width: number): [number, number, number, number][] {
  const s: [number, number, number, number][] = [];
  for (const y of [-34, 0, 34]) for (const z of [26, 56]) s.push([0, y, z, 30]);
  for (const f of fingerBoxes(width)) for (const dz of [-14, 6]) s.push([f.c[0], f.c[1], f.c[2] + dz, 14]);
  return s;
}
