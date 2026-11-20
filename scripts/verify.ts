/**
 * Verificación automática sin interfaz: ejecuta cada simulación completa y comprueba
 * IK, colisiones, distancia mínima entre robots, velocidades articulares (sin saltos) y el resultado final.
 *   npm run verify            (las tres)
 *   npm run verify -- torre   (solo una)
 */
import { Simulation, RobotActor } from '../src/sim/engine';
import { installCollision } from '../src/sim/collision';
import { buildKeychainScenario } from '../src/sim/scenarioKeychain';
import { buildTowerScenario } from '../src/sim/scenarioTower';
import { buildServiceScenario } from '../src/sim/scenarioService';

const only = process.argv[2];
let failed = false;
for (const build of [buildKeychainScenario, buildTowerScenario, buildServiceScenario]) {
  const sc = build();
  if (only && sc.id !== only) continue;
  const sim = new Simulation(sc);
  sim.logCap = 1e7;
  installCollision(sim);
  const maxQd = new Map<string, number>();
  let jumps = 0;
  const prev = new Map<string, number[]>();
  // la producción continua se prueba 330 s: visita automática a los 22 s y otra pedida "por botón" a los 150 s
  const limit = sc.continuous ? 330 : 400;
  while (!sim.finished && sim.time < limit) {
    sim.step();
    if (sc.continuous && Math.abs(sim.time - 150) < 1e-6 + 1 / 480) sim.signals.set('LLAMADA', true);
    for (const r of sc.robots as RobotActor[]) {
      const p = prev.get(r.id);
      if (p) {
        const d = Math.max(...r.q.map((v, i) => Math.abs(v - p[i])));
        if (d > 0.02) jumps++; // > 1,15° en 4 ms = 287°/s: sería un salto
        maxQd.set(r.id, Math.max(maxQd.get(r.id) ?? 0, d / (1 / 240)));
      }
      prev.set(r.id, r.q.slice());
    }
  }
  const errs = sim.log.filter((e) => e.kind === 'error');
  console.log(`\n=== ${sc.title} ===`);
  console.log(`tiempo de ciclo: ${sim.time.toFixed(1)} s · terminado: ${sim.finished}`);
  console.log(`distancia mínima UR5–Turin (envolventes): ${sim.minClear.toFixed(0)} mm`);
  for (const [id, v] of maxQd) console.log(`velocidad articular máxima ${id}: ${((v * 180) / Math.PI).toFixed(0)} °/s`);
  console.log(`saltos articulares: ${jumps}`);
  for (const e of errs) console.log(`  ✗ [${e.t.toFixed(2)} s] ${e.who}: ${e.text}`);
  if (sc.continuous) {
    const r = sc.checkResult(sim);
    console.log(`resultado: ${r.text} · ciclos de paletizado: ${sim.counters.get('ciclos') ?? 0}`);
    for (const e of sim.log.filter((x) => x.kind === 'success')) console.log(`  ✓ [${e.t.toFixed(1)} s] ${e.text}`);
    if (!(sim.counters.get('llaveros') === 2 && (sim.counters.get('ciclos') ?? 0) >= 2)) failed = true;
  } else {
    const last = sim.log[sim.log.length - 1];
    console.log(`resultado: ${last.text}`);
  }
  if (process.argv.includes('--log')) for (const e of sim.log) console.log(`  ${e.t.toFixed(2).padStart(6)} ${e.who.padEnd(13)} ${e.text}`);
  if (errs.length || (!sim.finished && !sc.continuous) || jumps) failed = true;
}
// Barrido: la visita se pide en distintos momentos del ciclo (el robot puede estar en cualquier punto de su programa)
if (!only || only === 'barrido') {
  console.log('');
  console.log('=== Barrido: pedido de visitante en distintos momentos del ciclo ===');
  for (let call = 26; call <= 230; call += 12) {
    const sc = buildServiceScenario();
    const sim = new Simulation(sc);
    sim.logCap = 1e6;
    installCollision(sim);
    // primera visita automática (22 s) incluida; la segunda se pide en `call` (si todavía hay una en curso, apenas termine)
    let asked = false;
    while (sim.time < 420 && (sim.counters.get('llaveros') ?? 0) < 2) {
      sim.step();
      if (!asked && sim.time >= call && !sc.customer!.present && !sim.signals.get('LLAMADA')) { sim.signals.set('LLAMADA', true); asked = true; }
    }
    const extra = 90; // y sigue paletizando un rato después de la entrega
    const t0 = sim.time;
    while (sim.time < t0 + extra) sim.step();
    const errs = sim.log.filter((e) => e.kind === 'error');
    const ok = !errs.length && sim.counters.get('llaveros') === 2;
    console.log(`  pedido a los ${String(call).padStart(3)} s → ${ok ? 'OK' : 'FALLA'} · llaveros ${sim.counters.get('llaveros') ?? 0} · torres ${sim.counters.get('torres') ?? 0} · ${errs.length} errores${errs.length ? ': ' + errs[0].text + ' @' + errs[0].t.toFixed(1) : ''}`);
    if (!ok) failed = true;
  }
}
process.exit(failed ? 1 : 0);
