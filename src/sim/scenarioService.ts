/**
 * SIMULACIÓN 3 — Producción continua + atención de un visitante.
 *
 * Los dos robots arman y desarman la torre GRUPRO sin parar (ciclo de la simulación 2).
 * Llega un visitante, pulsa el botón del pedestal (señal PEDIDO) y los robots lo atienden:
 *  - Cada robot termina la operación en curso y, en su próximo PUNTO SEGURO (pinza vacía y fuera de la zona de la
 *    torre), ejecuta su subprograma de servicio. Nunca se interrumpe con un cubo en la pinza ni dentro de la torre.
 *  - El UR toma el llavero del soporte y espera que el Turin esté listo; se lo pasa en la zona de transferencia.
 *  - El Turin lo lleva al visitante y se lo entrega en la mano. El pedido se da por atendido (PEDIDO = 0).
 *  - Ambos retoman el paletizado exactamente donde lo dejaron (las señales de turno no se tocaron).
 * El soporte se repone con otro llavero cuando el visitante se va. Con el botón "Llamar visitante" de la interfaz
 * se puede pedir otra visita en cualquier momento.
 */
import { type Pose, type Vec3, frameZY, pose } from '../kin/math';
import { CustomerActor, RobotActor, type Scenario, type SimObject, type Simulation, type Step } from './engine';
import { DOWN_JAW_Y, TABLE, above, grip, movej, movel, op, set, shift, tableStatic, wait, waitsig } from './common';
import { CUBE_OPEN, cubesBackInTrays, makeCubes, makeTowerCell, towerCheck, towerCycle, towerPreamble, towerSignals, towerStatics } from './scenarioTower';
import { KEYCHAIN } from './scenarioKeychain';

/** Soporte del llavero en el lado del UR, fuera de su bandeja de cubos. */
export const SERVICE_STAND = { c: [-720, -110, 12.5] as Vec3, h: [30, 20, 12.5] as Vec3 };
/** Pedestal con el pulsador para el visitante (sobre el piso, frente a la mesa del lado del Turin). */
export const BUTTON = { base: [860, -700, TABLE.floor] as Vec3, top: 250 };

export function buildServiceScenario(): Scenario {
  const cell = makeTowerCell();
  const cubes = makeCubes();

  const tagCenter: Vec3 = [SERVICE_STAND.c[0], SERVICE_STAND.c[1], 15 + KEYCHAIN.tag.hz];
  const keychain: SimObject = {
    id: 'llavero', kind: 'keychain', label: 'el llavero', pose: pose(tagCenter),
    half: [KEYCHAIN.tag.hx, KEYCHAIN.tag.hy, KEYCHAIN.tag.hz], holders: [],
  };

  const g = KEYCHAIN.urGrip, t = KEYCHAIN.turinGrip;
  const pick: Pose = { p: [tagCenter[0] + g[0], tagCenter[1] + g[1], tagCenter[2] + g[2]], R: DOWN_JAW_Y };
  // transferencia detrás de la torre, entre las dos bases, por encima de la altura de la torre armada
  const xfer: Vec3 = [-80, 220, 430];
  const urXfer: Pose = { p: [xfer[0] + g[0], xfer[1] + g[1], xfer[2] + g[2]], R: DOWN_JAW_Y };
  const tuXfer: Pose = { p: [xfer[0] + t[0], xfer[1] + t[1], xfer[2] + t[2]], R: frameZY([-1, 0, 0], [0, 1, 0]) };
  const tuDeliver: Pose = { p: [560, -470, 290], R: frameZY([0, -1, 0], [-1, 0, 0]) };
  const ring: Vec3 = [tuDeliver.p[0], tuDeliver.p[1] - (t[0] - KEYCHAIN.ring.c[0]), tuDeliver.p[2] + KEYCHAIN.ring.c[2]];
  const tuPre = shift(tuXfer, [100, 0, 0]);
  const tuDeliverPre = shift(tuDeliver, [0, 140, 0]);
  // puntos de paso en alto para los cambios grandes de orientación del Turin (evitan barrer la mesa y la torre)
  const tuVia1: Pose = { p: [110, 190, 500], R: frameZY([-1, 0, 0], [0, 1, 0]) };
  const tuVia2: Pose = { p: [430, -250, 480], R: frameZY([0, -1, 0], [-1, 0, 0]) };

  // ---------------- subprogramas de servicio (se ejecutan en el punto seguro si PEDIDO = 1)
  const urService: Step[] = [
    op('PEDIDO de llavero: atiende al visitante'),
    set('UR_FUERA', false),
    // el servicio arranca siempre desde Home: la trayectoria es la misma sin importar dónde se interrumpió el paletizado
    movej(cell.urHome, 'Pasa por Home (postura de referencia)'),
    grip(30, undefined, 'Abre la pinza a 30 mm para el llavero'),
    movej(above(pick, 120), 'Aproximación al soporte del llavero'),
    movel(pick, 'Bajada lineal a la posición de toma', 110),
    wait(0.25),
    grip(5, 'llavero', 'Cierra la pinza: sujeta el llavero'),
    wait(0.3),
    movel(above(pick, 120), 'Extrae el llavero del soporte', 120),
    set('UR_TOMO'),
    waitsig('TURIN_LISTO', 'Espera que el Turin esté listo para recibir'),
    movej(above(urXfer, 140), 'Traslado hacia la zona de transferencia'),
    movel(urXfer, 'Bajada lineal a la posición de transferencia', 120),
    set('UR_EN_TRANSFER'),
    waitsig('TURIN_SUJETA', 'Sostiene el llavero quieto: espera al Turin'),
    wait(0.35, 'Pausa de seguridad: ambos sujetan el llavero'),
    grip(30, undefined, 'Abre la pinza: cede el llavero'),
    wait(0.2),
    movel(above(urXfer, 140), 'Se retira de la zona de transferencia', 150),
    set('UR_FUERA'),
    set('UR_EN_TRANSFER', false),
    set('UR_TOMO', false),
    movej(cell.urHome, 'Vuelve a Home (postura de referencia del paletizado)'),
    grip(CUBE_OPEN, undefined, 'Pinza para cubos (82 mm)'),
    op('Retoma el paletizado donde lo dejó'),
  ];
  const tuService: Step[] = [
    op('PEDIDO de llavero: va a recibirlo del UR'),
    set('TURIN_SOLTO', false),
    movej(cell.tuHome, 'Pasa por Home (postura de referencia)'),
    grip(30, undefined, 'Abre la pinza a 30 mm para el llavero'),
    waitsig('UR_TOMO', 'Espera que el UR tome el llavero (zona de transferencia libre)'),
    movej(tuVia1, 'Gira la herramienta a horizontal, en alto'),
    movej(tuPre, 'Preposiciona frente a la zona de transferencia'),
    set('TURIN_LISTO'),
    waitsig('UR_EN_TRANSFER', 'Espera el llavero en la transferencia'),
    movel(tuXfer, 'Aproximación lineal horizontal para recibir', 90),
    wait(0.25),
    grip(5, 'llavero', 'Cierra la pinza sobre el extremo libre del llavero'),
    set('TURIN_SUJETA'),
    waitsig('UR_FUERA', 'Espera que el UR libere y se retire'),
    set('TURIN_LISTO', false),
    movel(shift(tuXfer, [100, 0, 0]), 'Retira el llavero de la zona de transferencia', 140),
    movej(tuVia2, 'Gira hacia el visitante, en alto'),
    movej(tuDeliverPre, 'Se ubica frente al visitante'),
    movel(tuDeliver, 'Extiende el brazo y presenta el llavero', 110),
    set('EN_ENTREGA'),
    waitsig('CLIENTE_TOMO', 'Espera que el visitante tome el llavero'),
    wait(0.35),
    grip(30, undefined, 'Abre la pinza: entrega el llavero'),
    set('TURIN_SOLTO'),
    set('EN_ENTREGA', false),
    set('TURIN_SUJETA', false),
    set('PEDIDO', false),
    movel(tuDeliverPre, 'Se retira del visitante', 140),
    movej(tuVia2, 'Sube a la posición intermedia'),
    movej(cell.tuHome, 'Vuelve a Home (postura de referencia del paletizado)'),
    grip(CUBE_OPEN, undefined, 'Pinza para cubos (82 mm)'),
    op('Pedido atendido: retoma el paletizado'),
  ];
  // Punto seguro: se atiende el pedido solo si además NO es mi turno en la torre. Así el otro robot (que quizá ya
  // tiene un cubo en la pinza esperando su turno) puede terminar su operación y llegar a su propio punto seguro.
  // Si es mi turno, hago una operación más y lo atiendo en el punto seguro siguiente.
  const safe = (who: 'ur' | 'turin', service: Step[]): Step[] => [{
    k: 'if', sig: 'PEDIDO', value: true, then: service, text: 'Punto seguro: hay un PEDIDO pendiente, interrumpe el paletizado',
    also: { a: 'UR_FIN_TURNO', b: 'TURIN_FIN_TURNO', equal: who === 'turin' },
  }];

  const robotProg = (who: 'ur' | 'turin', service: Step[]): Step[] => {
    const head: Step[] = [op('Producción continua: paletizado GRUPRO'), grip(CUBE_OPEN, undefined, 'Pinza para cubos (82 mm)'), ...towerPreamble(who)];
    const loopStart = head.length;
    return [...head, ...towerCycle(who, cell, safe(who, service)), { k: 'check', id: `ciclo_${who}` }, { k: 'jump', to: loopStart }];
  };

  // ---------------- visitante
  const offstage: Vec3 = [2300, -1700, TABLE.floor];
  const atButton: Vec3 = [560, -880, TABLE.floor];
  const buttonCap: Vec3 = [BUTTON.base[0], BUTTON.base[1], BUTTON.top + 14];
  const visit: Step[] = [
    { k: 'present', value: true },
    op('Un visitante se acerca al stand'),
    { k: 'walk', to: atButton, t: 4.2, text: 'Camina hasta el pedestal del botón' },
    { k: 'reach', to: [buttonCap[0] - 10, buttonCap[1] - 10, buttonCap[2] + 30], t: 1.1, text: 'Pulsa el botón "QUIERO MI LLAVERO"' },
    set('PEDIDO'),
    wait(0.35),
    { k: 'reach', to: 'rest', t: 0.9 },
    waitsig('EN_ENTREGA', 'Mira cómo los robots preparan su llavero'),
    { k: 'reach', to: [ring[0] + 8, ring[1] - 4, ring[2] - 6], t: 1.4, text: 'Extiende la mano hacia el llavero' },
    { k: 'hand', close: true, obj: 'llavero', text: 'Toma el llavero' },
    set('CLIENTE_TOMO'),
    waitsig('TURIN_SOLTO', 'Espera que la pinza se abra'),
    wait(0.3),
    { k: 'reach', to: 'chest', t: 1.4, text: 'Se lleva el llavero' },
    set('CLIENTE_TOMO', false),
    { k: 'check', id: 'llavero' },
    wait(0.8),
    { k: 'walk', to: [1400, -1500, TABLE.floor], t: 3.6, text: 'Se retira del stand con su llavero' },
    { k: 'hand', close: false },
    { k: 'present', value: false },
    { k: 'respawn', obj: 'llavero', text: 'Reposición: llavero nuevo en el soporte' },
    set('LLAMADA', false),
  ];
  // primera visita automática a los 22 s; luego, cada vez que se pulse "Llamar visitante" (señal LLAMADA)
  const custProg: Step[] = [{ k: 'present', value: false }, wait(22), ...visit];
  const waitCallIdx = custProg.length;
  custProg.push(op('Nadie en el stand'), waitsig('LLAMADA', 'Esperando visitantes'), ...visit, { k: 'jump', to: waitCallIdx });

  return {
    id: 'servicio',
    title: 'Producción continua + visitante',
    continuous: true,
    robots: [
      new RobotActor('ur', 'UR5', cell.ur, cell.urHome, robotProg('ur', urService)),
      new RobotActor('turin', 'Turin TCR050', cell.tu, cell.tuHome, robotProg('turin', tuService)),
    ],
    customer: new CustomerActor('cliente', 'Visitante', offstage, custProg),
    objects: [...cubes, keychain],
    initialPoses: { llavero: pose(tagCenter) },
    signals: [
      { id: 'PEDIDO', label: 'PEDIDO', from: 'Botón', to: 'UR DI3 + Turin DI3' },
      ...towerSignals,
      { id: 'UR_TOMO', label: 'UR_TOMO', from: 'UR DO6', to: 'Turin DI7' },
      { id: 'TURIN_LISTO', label: 'TURIN_LISTO', from: 'Turin DO4', to: 'UR DI4' },
      { id: 'UR_EN_TRANSFER', label: 'UR_EN_TRANSFER', from: 'UR DO4', to: 'Turin DI4' },
      { id: 'TURIN_SUJETA', label: 'TURIN_SUJETA', from: 'Turin DO5', to: 'UR DI5' },
      { id: 'UR_FUERA', label: 'UR_FUERA', from: 'UR DO5', to: 'Turin DI5' },
      { id: 'EN_ENTREGA', label: 'EN_ENTREGA', from: 'Turin DO6', to: 'Visitante' },
      { id: 'CLIENTE_TOMO', label: 'CLIENTE_TOMO', from: 'Sensor mano', to: 'Turin DI6' },
      { id: 'TURIN_SOLTO', label: 'TURIN_SOLTO', from: 'Turin DO7', to: 'Visitante' },
      { id: 'LLAMADA', label: 'LLAMADA (interfaz)', from: 'Botón de la interfaz', to: 'Visitante' },
    ],
    statics: [tableStatic, ...towerStatics, { id: 'el soporte del llavero', c: SERVICE_STAND.c, h: SERVICE_STAND.h }],
    checks: {
      torre: towerCheck,
      // al terminar su ciclo, cada robot comprueba que SUS tres cubos estén en su lugar de la bandeja
      ciclo_ur: (sim: Simulation) => cubesBackInTrays(sim, [0, 2, 4], 'UR5', 'ciclos'),
      ciclo_turin: (sim: Simulation) => cubesBackInTrays(sim, [1, 3, 5], 'Turin'),
      llavero: (sim: Simulation) => {
        const o = sim.obj('llavero');
        const ok = o.holders.length === 1 && o.holders[0].by === 'cliente';
        return ok ? { ok, text: '¡Llavero GRUPRO entregado al visitante!', counter: 'llaveros' } : { ok, text: 'El llavero no llegó al visitante' };
      },
    },
    checkResult: (sim: Simulation) => ({
      ok: sim.errors === 0,
      text: `${sim.counters.get('torres') ?? 0} torres, ${sim.counters.get('llaveros') ?? 0} llaveros entregados`,
    }),
  };
}
