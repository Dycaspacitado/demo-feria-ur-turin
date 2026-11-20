/**
 * SIMULACIÓN 1 — Transferencia y entrega de un llavero.
 *
 * El UR5 toma el llavero de su soporte, lo lleva a la zona de transferencia y lo sostiene por el lado del aro.
 * El Turin se acerca en horizontal y lo toma por el otro extremo (zonas de agarre distintas: los dedos no se cruzan).
 * El UR abre, se retira y avisa; el Turin gira hacia el cliente, extiende el brazo y lo entrega en su mano.
 *
 * Coordinación (señales digitales, como se cablearía entre los dos controladores):
 *   UR  DO1 → Turin DI1  UR_TOMO          el UR ya tiene el llavero (el Turin puede preposicionarse)
 *   UR  DO2 → Turin DI2  UR_EN_TRANSFER   llavero quieto en la posición de transferencia
 *   Turin DO1 → UR DI1   TURIN_SUJETA     el Turin cerró su pinza sobre el llavero
 *   UR  DO3 → Turin DI3  UR_FUERA         el UR abrió y salió de la zona de transferencia
 *   Turin DO2 → Sensor   EN_ENTREGA       llavero presentado al cliente
 *   Sensor → Turin DI4   CLIENTE_TOMO     el cliente lo tomó (barrera/sensor de presencia en la mano)
 *   Turin DO3            TURIN_SOLTO      pinza abierta, el cliente se lo lleva
 */
import { type Pose, type Vec3, frameZY, pose } from '../kin/math';
import { CustomerActor, RobotActor, type Scenario, type SimObject, type Simulation, type Step } from './engine';
import {
  DOWN_JAW_Y, TABLE, above, grip, makeKin, movej, movel, op, set, shift, solveHome, tableStatic, wait, waitsig,
} from './common';

export const KEYCHAIN = {
  tag: { hx: 40, hy: 2.5, hz: 17.5 },
  ring: { c: [-44, 0, 8] as Vec3, r: 11, tube: 1.3 },
  /** Puntos de agarre en el marco del llavero */
  urGrip: [-14, 0, 9] as Vec3,
  turinGrip: [18, 0, 0] as Vec3,
};
export const STAND = { c: [-380, -250, 12.5] as Vec3, h: [30, 20, 12.5] as Vec3 };

export function buildKeychainScenario(): Scenario {
  const ur = makeKin('UR5', [-480, 150, TABLE.top], 0);
  const tu = makeKin('TCR050', [480, 150, TABLE.top], 180);

  // ---- objeto: el llavero (placa acrílica 80 × 35 × 5 mm con aro), parado en el soporte
  const tagCenter: Vec3 = [STAND.c[0], STAND.c[1], 15 + KEYCHAIN.tag.hz];
  const keychain: SimObject = {
    id: 'llavero', kind: 'keychain', label: 'el llavero', pose: pose(tagCenter),
    half: [KEYCHAIN.tag.hx, KEYCHAIN.tag.hy, KEYCHAIN.tag.hz], holders: [],
  };

  // ---- puntos de la tarea
  const g = KEYCHAIN.urGrip, t = KEYCHAIN.turinGrip;
  const pick: Pose = { p: [tagCenter[0] + g[0], tagCenter[1] + g[1], tagCenter[2] + g[2]], R: DOWN_JAW_Y };
  const xfer: Vec3 = [0, 150, 330]; // centro del llavero durante la transferencia
  const urXfer: Pose = { p: [xfer[0] + g[0], xfer[1] + g[1], xfer[2] + g[2]], R: DOWN_JAW_Y };
  const R_tu_xfer = frameZY([-1, 0, 0], [0, 1, 0]); // horizontal, apuntando al UR
  const tuXfer: Pose = { p: [xfer[0] + t[0], xfer[1] + t[1], xfer[2] + t[2]], R: R_tu_xfer };
  const R_deliver = frameZY([0, -1, 0], [-1, 0, 0]); // horizontal hacia el cliente, placa vertical
  const tuDeliver: Pose = { p: [520, -540, 260], R: R_deliver };
  const ringAtDelivery: Vec3 = [tuDeliver.p[0], tuDeliver.p[1] - (t[0] - KEYCHAIN.ring.c[0]), tuDeliver.p[2] + KEYCHAIN.ring.c[2]];

  const urHome = solveHome(ur, { p: [-300, -150, 400], R: DOWN_JAW_Y });
  const tuHome = solveHome(tu, { p: [300, -150, 400], R: DOWN_JAW_Y });

  const urProg: Step[] = [
    op('Inicio de ciclo: pinza abierta en Home'),
    grip(30, undefined, 'Pre-apertura de pinza (30 mm)'),
    movej(above(pick, 120), 'Aproximación sobre el soporte del llavero'),
    movel(pick, 'Bajada lineal a la posición de toma', 110),
    wait(0.3),
    grip(5, 'llavero', 'Cierra la pinza: sujeta el llavero'),
    wait(0.35),
    movel(above(pick, 120), 'Extrae el llavero del soporte', 120),
    set('UR_TOMO'),
    movej(above(urXfer, 140), 'Traslado hacia la zona de transferencia'),
    movel(urXfer, 'Bajada lineal a la posición de transferencia', 120),
    set('UR_EN_TRANSFER'),
    waitsig('TURIN_SUJETA', 'Sostiene el llavero quieto: espera al Turin'),
    wait(0.4, 'Pausa de seguridad: ambos sujetan el llavero'),
    grip(30, undefined, 'Abre la pinza: cede el llavero'),
    wait(0.25),
    movel(above(urXfer, 140), 'Se retira hacia arriba de la zona de transferencia', 150),
    set('UR_FUERA'),
    movej(urHome, 'Regresa a Home'),
    set('UR_TOMO', false),
    set('UR_EN_TRANSFER', false),
    op('En Home: listo para el próximo cliente'),
  ];

  const tuPre = shift(tuXfer, [150, 0, 0]);
  const tuDeliverPre = shift(tuDeliver, [0, 140, 0]);
  const tuProg: Step[] = [
    op('En Home: espera que el UR tome el llavero'),
    waitsig('UR_TOMO', 'Espera señal UR_TOMO'),
    grip(30, undefined, 'Pre-apertura de pinza (30 mm)'),
    movej(tuPre, 'Preposiciona frente a la zona de transferencia'),
    waitsig('UR_EN_TRANSFER', 'Espera que el llavero esté quieto en la transferencia'),
    movel(tuXfer, 'Aproximación lineal horizontal para recibir', 90),
    wait(0.3),
    grip(5, 'llavero', 'Cierra la pinza sobre el extremo libre del llavero'),
    set('TURIN_SUJETA'),
    waitsig('UR_FUERA', 'Espera que el UR libere y se retire'),
    movel(shift(tuXfer, [140, 0, 0]), 'Retira el llavero de la zona de transferencia', 140),
    movej(tuDeliverPre, 'Gira hacia el cliente'),
    movel(tuDeliver, 'Extiende el brazo y presenta el llavero', 110),
    set('EN_ENTREGA'),
    waitsig('CLIENTE_TOMO', 'Espera que el cliente tome el llavero'),
    wait(0.4),
    grip(30, undefined, 'Abre la pinza: entrega el llavero'),
    set('TURIN_SOLTO'),
    movel(tuDeliverPre, 'Se retira del cliente', 140),
    movej(tuHome, 'Regresa a Home'),
    set('TURIN_SUJETA', false),
    set('EN_ENTREGA', false),
    op('En Home: entrega completada'),
  ];

  const feet: Vec3 = [470, -960, TABLE.floor];
  const custProg: Step[] = [
    op('Mira la demostración frente al stand'),
    waitsig('EN_ENTREGA', 'Espera su llavero'),
    { k: 'reach', to: [ringAtDelivery[0] + 8, ringAtDelivery[1] - 4, ringAtDelivery[2] - 6], t: 1.4, text: 'Extiende la mano hacia el llavero' },
    { k: 'hand', close: true, obj: 'llavero', text: 'Toma el llavero' },
    set('CLIENTE_TOMO'),
    waitsig('TURIN_SOLTO', 'Espera que la pinza se abra'),
    wait(0.3),
    { k: 'reach', to: 'chest', t: 1.6, text: 'Se lleva el llavero' },
    set('CLIENTE_TOMO', false),
    op('¡Llavero GRUPRO entregado!'),
  ];

  return {
    id: 'llavero',
    title: 'Transferencia y entrega del llavero',
    robots: [new RobotActor('ur', 'UR5', ur, urHome, urProg), new RobotActor('turin', 'Turin TCR050', tu, tuHome, tuProg)],
    customer: new CustomerActor('cliente', 'Cliente', feet, custProg),
    objects: [keychain],
    signals: [
      { id: 'UR_TOMO', label: 'UR_TOMO', from: 'UR DO1', to: 'Turin DI1' },
      { id: 'UR_EN_TRANSFER', label: 'UR_EN_TRANSFER', from: 'UR DO2', to: 'Turin DI2' },
      { id: 'TURIN_SUJETA', label: 'TURIN_SUJETA', from: 'Turin DO1', to: 'UR DI1' },
      { id: 'UR_FUERA', label: 'UR_FUERA', from: 'UR DO3', to: 'Turin DI3' },
      { id: 'EN_ENTREGA', label: 'EN_ENTREGA', from: 'Turin DO2', to: 'Panel/cliente' },
      { id: 'CLIENTE_TOMO', label: 'CLIENTE_TOMO', from: 'Sensor mano', to: 'Turin DI4' },
      { id: 'TURIN_SOLTO', label: 'TURIN_SOLTO', from: 'Turin DO3', to: 'Panel/cliente' },
    ],
    statics: [tableStatic, { id: 'el soporte del llavero', c: STAND.c, h: STAND.h }],
    checkResult: (sim: Simulation) => {
      const o = sim.obj('llavero');
      const inHand = o.holders.length === 1 && o.holders[0].by === 'cliente';
      return { ok: inHand, text: inHand ? 'el cliente tiene su llavero' : 'el llavero NO llegó al cliente' };
    },
  };
}
