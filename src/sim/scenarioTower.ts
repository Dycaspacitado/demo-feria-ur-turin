/**
 * SIMULACIÓN 2 — Paletizado de cubos GRUPRO: armado de la torre y desarmado en orden inverso.
 *
 * Seis cubos de 60 mm con una letra en la cara frontal. Leída de arriba hacia abajo la torre dice G-R-U-P-R-O,
 * así que se apila de abajo hacia arriba: O, R, P, U, R, G. El UR coloca los niveles impares (O, P, R)
 * y el Turin los pares (R, U, G), cada uno desde su bandeja de alimentación.
 * Con la torre completa se hace el control de calidad; después se desarma de arriba hacia abajo (G, R, U, P, R, O)
 * y cada cubo vuelve a SU lugar en la bandeja, así la celda queda lista para el próximo ciclo.
 *
 * Exclusión de la zona de la torre (protocolo de bit alternado, cada robot escribe SOLO sus salidas):
 *   UR  DO1 → Turin DI1  UR_FIN_TURNO      se invierte cada vez que el UR termina una operación en la torre
 *   Turin DO1 → UR DI1   TURIN_FIN_TURNO   se invierte cada vez que el Turin termina una operación en la torre
 *   El UR puede entrar cuando ambos bits son iguales; el Turin, cuando son distintos.
 *   UR  DO2 / Turin DO2  *_EN_ZONA         redundancia de seguridad: nadie entra si el otro está dentro.
 * El Turin coloca la G y, tras la pausa de exhibición, la retira él mismo sin ceder el turno (es el cubo de arriba).
 * Mientras uno trabaja en la torre, el otro toma o devuelve su cubo en la bandeja: trabajan al mismo tiempo.
 */
import { type Pose, type Vec3, pose } from '../kin/math';
import { type RobotKin } from '../kin/robot';
import { RobotActor, type Scenario, type SimObject, type Simulation, type Step } from './engine';
import { DOWN_JAW_X, TABLE, above, grip, makeKin, movej, movel, op, set, solveHome, tableStatic, wait, waitsig } from './common';

export const CUBE = 60;
export const FIXTURE = { c: [0, -120, 5] as Vec3, h: [70, 70, 5] as Vec3 };
export const TRAYS = {
  ur: { c: [-440, -260, 5] as Vec3, h: [190, 55, 5] as Vec3 },
  turin: { c: [440, -260, 5] as Vec3, h: [190, 55, 5] as Vec3 },
};
/** Apilado de abajo hacia arriba (se lee GRUPRO de arriba hacia abajo). */
export const STACK = ['O', 'R', 'P', 'U', 'R', 'G'];
export const CUBE_OPEN = 82;
const CLOSED = CUBE;
const TRAY_X = { ur: [-320, -440, -560], turin: [320, 440, 560] };

export const levelCenter = (k: number): Vec3 => [FIXTURE.c[0], FIXTURE.c[1], 10 + CUBE / 2 + CUBE * k];
const slotPose = (level: number): Pose => {
  const who = level % 2 === 0 ? 'ur' : 'turin';
  return pose([TRAY_X[who][Math.floor(level / 2)], -260, 10 + CUBE / 2]);
};

export function makeCubes(): SimObject[] {
  return STACK.map((letter, level) => ({
    id: `cubo${level}`, kind: 'cube', letter, label: `el cubo ${letter} (nivel ${level + 1})`,
    pose: slotPose(level), half: [CUBE / 2, CUBE / 2, CUBE / 2], holders: [],
  }));
}

export interface TowerCell {
  ur: RobotKin;
  tu: RobotKin;
  urHome: number[];
  tuHome: number[];
  waitPose: { ur: Pose; turin: Pose };
}

export function makeTowerCell(): TowerCell {
  const ur = makeKin('UR5', [-450, 200, TABLE.top], 0);
  const tu = makeKin('TCR050', [450, 200, TABLE.top], 180);
  return {
    ur, tu,
    urHome: solveHome(ur, { p: [-300, -40, 420], R: DOWN_JAW_X }),
    tuHome: solveHome(tu, { p: [300, -40, 420], R: DOWN_JAW_X }),
    waitPose: { ur: { p: [-250, -170, 330], R: DOWN_JAW_X }, turin: { p: [250, -170, 330], R: DOWN_JAW_X } },
  };
}

export const towerSignals = [
  { id: 'UR_FIN_TURNO', label: 'UR_FIN_TURNO', from: 'UR DO1', to: 'Turin DI1' },
  { id: 'TURIN_FIN_TURNO', label: 'TURIN_FIN_TURNO', from: 'Turin DO1', to: 'UR DI1' },
  { id: 'UR_EN_ZONA', label: 'UR_EN_ZONA', from: 'UR DO2', to: 'Turin DI2' },
  { id: 'TURIN_EN_ZONA', label: 'TURIN_EN_ZONA', from: 'Turin DO2', to: 'UR DI2' },
];

export const towerStatics = [
  { id: 'la base de la torre', c: FIXTURE.c, h: FIXTURE.h },
  { id: 'la bandeja del UR', c: TRAYS.ur.c, h: TRAYS.ur.h },
  { id: 'la bandeja del Turin', c: TRAYS.turin.c, h: TRAYS.turin.h },
];

/**
 * Programa de un robot para un ciclo completo (armar + desarmar).
 * `safePoint` se inserta cada vez que el robot queda con la pinza vacía y fuera de la zona de la torre
 * (ahí se puede atender un pedido sin dejar la torre a medias ni un cubo en el aire).
 */
export function towerCycle(who: 'ur' | 'turin', cell: TowerCell, safePoint: Step[] = []): Step[] {
  const mine = who === 'ur' ? 'UR_FIN_TURNO' : 'TURIN_FIN_TURNO';
  const myZone = who === 'ur' ? 'UR_EN_ZONA' : 'TURIN_EN_ZONA';
  const otherZone = who === 'ur' ? 'TURIN_EN_ZONA' : 'UR_EN_ZONA';
  const waitPose = cell.waitPose[who];
  const myLevels = who === 'ur' ? [0, 2, 4] : [1, 3, 5];
  const L = (lv: number) => `${STACK[lv]}" (nivel ${lv + 1})`;

  const waitTurn = (lv: number, verb: string): Step[] => [
    { k: 'waitcmp', a: 'UR_FIN_TURNO', b: 'TURIN_FIN_TURNO', equal: who === 'ur', text: `Espera su turno para ${verb} el nivel ${lv + 1}` },
    waitsig(otherZone, 'Comprueba que el otro robot esté fuera de la zona', false),
    set(myZone),
  ];
  const leaveZone = (toggle: boolean): Step[] => [
    movej(waitPose, 'Sale de la zona de la torre'),
    set(myZone, false),
    ...(toggle ? [{ k: 'toggle', sig: mine } as Step] : []),
  ];
  const pickFrom = (P: Pose, lv: number, where: string): Step[] => [
    movej(above(P, 110), `Aproximación al cubo "${L(lv)} ${where}`),
    movel(P, `Baja a tomar el cubo "${STACK[lv]}"`, 110),
    wait(0.2),
    grip(CLOSED, `cubo${lv}`, `Cierra la pinza sobre el cubo "${STACK[lv]}"`),
    wait(0.25),
    movel(above(P, 110), `Levanta el cubo "${STACK[lv]}"`, 150),
  ];
  const placeAt = (P: Pose, lv: number, what: string, speed = 70): Step[] => [
    movej(above(P, 100), `Sobre ${what}`),
    movel(P, `Apoya el cubo "${STACK[lv]}" ${what}`, speed),
    wait(0.25),
    grip(CUBE_OPEN, undefined, 'Abre la pinza: cubo depositado'),
    wait(0.2),
    movel(above(P, 100), 'Sale hacia arriba sin tocar los cubos', 150),
  ];
  const tower = (lv: number): Pose => ({ p: levelCenter(lv), R: DOWN_JAW_X });
  const slot = (lv: number): Pose => ({ p: slotPose(lv).p, R: DOWN_JAW_X });

  const prog: Step[] = [op('ARMADO de la torre GRUPRO')];
  if (who === 'turin') {
    // El Turin arranca cada ciclo con el turno (lo dejó así el ciclo anterior, o el preámbulo del programa)
    // y lo cede para que el UR coloque la base. Así cada robot invierte su bit 6 veces por ciclo
    // y el estado de las señales al final es idéntico al del principio: el ciclo se puede repetir sin fin.
    prog.push(
      { k: 'waitcmp', a: 'UR_FIN_TURNO', b: 'TURIN_FIN_TURNO', equal: false, text: 'Tiene el turno al iniciar el ciclo' },
      { k: 'toggle', sig: mine },
      op('Cede el turno al UR para colocar la base "O"'),
      ...safePoint,
    );
  }
  // ---- armado: O R P U R G (de abajo hacia arriba)
  for (const lv of myLevels) {
    prog.push(
      ...pickFrom(slot(lv), lv, 'en la bandeja'),
      movej(waitPose, 'Lleva el cubo al punto de espera, fuera de la zona de la torre'),
      ...waitTurn(lv, 'colocar'),
      ...placeAt(tower(lv), lv, `alineado en el nivel ${lv + 1} de la torre`),
    );
    if (lv === 5) {
      // el Turin coloca la G: torre completa. Control de calidad y exhibición; luego la retira él mismo.
      prog.push(...leaveZone(false), { k: 'check', id: 'torre' }, wait(3.0, 'Torre GRUPRO completa: pausa de exhibición'));
      prog.push(op('DESARMADO de la torre: de arriba hacia abajo'), set(myZone), ...pickFrom(tower(5), 5, 'de la cima de la torre'), ...leaveZone(true));
      prog.push(...placeAt(slot(5), 5, 'en su lugar de la bandeja', 110), ...safePoint);
    } else {
      prog.push(...leaveZone(true), ...safePoint);
    }
  }
  // ---- desarmado: R U P R O (G ya la retiró el Turin); cada cubo vuelve a su lugar de la bandeja
  if (who === 'ur') prog.push(op('DESARMADO de la torre: de arriba hacia abajo'));
  for (const lv of [...myLevels].reverse()) {
    if (lv === 5) continue;
    prog.push(
      ...waitTurn(lv, 'retirar'),
      ...pickFrom(tower(lv), lv, 'de la cima de la torre'),
      ...leaveZone(true),
      ...placeAt(slot(lv), lv, 'en su lugar de la bandeja', 110),
      ...safePoint,
    );
  }
  return prog;
}

/** Preámbulo (una sola vez al encender): el Turin toma el turno inicial, que en cada ciclo cede al UR. */
export const towerPreamble = (who: 'ur' | 'turin'): Step[] => (who === 'turin' ? [{ k: 'toggle', sig: 'TURIN_FIN_TURNO' }] : []);

export function buildTowerScenario(): Scenario {
  const cell = makeTowerCell();
  const cubes = makeCubes();
  const prog = (who: 'ur' | 'turin', home: number[]): Step[] => [
    op('Inicio de ciclo en Home'),
    grip(CUBE_OPEN, undefined, 'Apertura de pinza para cubos (82 mm)'),
    ...towerPreamble(who),
    ...towerCycle(who, cell),
    movej(home, 'Regresa a Home: posición de reposo'),
    op('En reposo: cubos de vuelta en las bandejas'),
  ];
  return {
    id: 'torre',
    title: 'Paletizado de cubos GRUPRO',
    robots: [
      new RobotActor('ur', 'UR5', cell.ur, cell.urHome, prog('ur', cell.urHome)),
      new RobotActor('turin', 'Turin TCR050', cell.tu, cell.tuHome, prog('turin', cell.tuHome)),
    ],
    objects: cubes,
    signals: towerSignals,
    statics: [tableStatic, ...towerStatics],
    checks: { torre: towerCheck },
    checkResult: (sim: Simulation) => {
      const built = sim.counters.get('torres') ?? 0;
      const back = cubesBackInTrays(sim);
      return built === 1 && back.ok
        ? { ok: true, text: `torre GRUPRO armada y desarmada; ${back.text}` }
        : { ok: false, text: built !== 1 ? 'la torre no se completó' : back.text };
    },
  };
}

/** Control de calidad con la torre armada: orden de letras y alineación de cada nivel. */
export function towerCheck(sim: Simulation) {
  let worst = 0;
  for (let level = 0; level < 6; level++) {
    const o = sim.obj(`cubo${level}`);
    if (o.holders.length) return { ok: false, text: `${o.label} sigue en una pinza` };
    const e = levelCenter(level);
    worst = Math.max(worst, Math.hypot(o.pose.p[0] - e[0], o.pose.p[1] - e[1], o.pose.p[2] - e[2]));
  }
  const word = [...STACK].reverse().join('');
  return worst < 1
    ? { ok: true, text: `¡Torre "${word}" completa! Desalineación máxima ${worst.toFixed(2)} mm`, counter: 'torres' }
    : { ok: false, text: `Torre desalineada (${worst.toFixed(1)} mm)` };
}

export function cubesBackInTrays(sim: Simulation, levels = [0, 1, 2, 3, 4, 5], who = '', counter?: string) {
  let worst = 0;
  for (const level of levels) {
    const o = sim.obj(`cubo${level}`);
    if (o.holders.length) return { ok: false, text: `${o.label} sigue en una pinza` };
    const e = slotPose(level).p;
    worst = Math.max(worst, Math.hypot(o.pose.p[0] - e[0], o.pose.p[1] - e[1], o.pose.p[2] - e[2]));
  }
  const n = levels.length === 6 ? 'los 6 cubos volvieron' : `${who}: sus ${levels.length} cubos volvieron`;
  return worst < 1 ? { ok: true, text: `${n} a su lugar de la bandeja (error ${worst.toFixed(2)} mm)`, counter } : { ok: false, text: `cubos fuera de lugar (${worst.toFixed(1)} mm)` };
}
