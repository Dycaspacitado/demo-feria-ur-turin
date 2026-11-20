/**
 * Motor de simulación (sin Three.js, se puede ejecutar en Node).
 *
 * Cada actor (UR5, Turin, cliente) ejecuta su propio programa de pasos, igual que un controlador real.
 * La sincronización se hace SOLO con señales digitales compartidas (handshake), como se cablearía en la feria.
 * Paso fijo de 1/240 s: la velocidad de reproducción solo cambia cuántos pasos se ejecutan por fotograma.
 */
import {
  type Pose, type Vec3, MINJERK_APEAK, MINJERK_VPEAK, matToQuat, minJerk, mul3, norm, poseInv, poseMul,
  quatToMat, rotAngle, slerp, sub, lerp3, pose,
} from '../kin/math';
import { RobotKin, ikNear } from '../kin/robot';
import { GRIPPER } from './gripper';

export const DT = 1 / 240;

// ------------------------------------------------------------------ objetos manipulados
export interface SimObject {
  id: string;
  kind: 'keychain' | 'cube';
  label: string;
  letter?: string;
  pose: Pose;
  /** Semiejes de la caja envolvente (marco del objeto), para contactos y colisiones. */
  half: Vec3;
  /** Quién lo sujeta (puede haber dos durante la transferencia); el último define la pose. */
  holders: { by: string; rel: Pose }[];
}

// ------------------------------------------------------------------ pasos de programa
export type Step =
  | { k: 'op'; text: string }
  | { k: 'movej'; to: Pose | number[]; speed?: number; text?: string }
  | { k: 'movel'; to: Pose; speed?: number; text?: string }
  | { k: 'grip'; width: number; obj?: string; text?: string }
  | { k: 'wait'; t: number; text?: string }
  | { k: 'set'; sig: string; value: boolean }
  | { k: 'waitsig'; sig: string; value: boolean; text?: string }
  | { k: 'toggle'; sig: string }
  | { k: 'waitcmp'; a: string; b: string; equal: boolean; text?: string }
  | { k: 'reach'; to: Vec3 | 'rest' | 'chest'; t: number; text?: string } // solo cliente
  | { k: 'hand'; close: boolean; obj?: string; text?: string } // solo cliente
  | { k: 'walk'; to: Vec3; t: number; text?: string } // solo cliente: camina hasta `to` (pies) y queda mirando a la mesa
  | { k: 'present'; value: boolean } // solo cliente: entra o sale de escena
  /** Punto seguro de un programa: si la señal tiene ese valor, ejecuta el subprograma `then` y después sigue. */
  | { k: 'if'; sig: string; value: boolean; then: Step[]; text?: string; also?: { a: string; b: string; equal: boolean } }
  /** Salto a un índice del programa en curso (para ciclos continuos). */
  | { k: 'jump'; to: number }
  /** Repone un objeto en su posición inicial (p. ej. un llavero nuevo en el soporte). */
  | { k: 'respawn'; obj: string; text?: string }
  /** Control de calidad programado: llama a scenario.checks[id] y lo registra (y suma al contador). */
  | { k: 'check'; id: string };

export type ActorState = 'listo' | 'moviendo' | 'lineal' | 'pinza' | 'esperando' | 'pausa' | 'terminado' | 'error';

export interface EventEntry { seq: number; t: number; who: string; text: string; kind: 'info' | 'signal' | 'warn' | 'error' | 'success' }

interface Running {
  step: Step;
  t: number;
  dur: number;
  q0?: number[];
  q1?: number[];
  p0?: Pose;
  p1?: Pose;
  w0?: number;
  h0?: Vec3;
  h1?: Vec3;
  f0?: Vec3;
  f1?: Vec3;
  y0?: number;
  yDir?: number;
  /** Ya se informó un error de alcance en este paso (para no repetirlo en cada ciclo). */
  warned?: boolean;
}

abstract class Actor {
  state: ActorState = 'listo';
  op = 'En reposo';
  run: Running | null = null;
  /** Pila de programas: el principal abajo; los subprogramas de un `if` se apilan encima. */
  frames: { prog: Step[]; pc: number }[];
  constructor(readonly id: string, readonly label: string, program: Step[]) {
    this.frames = [{ prog: program, pc: 0 }];
  }
  get program() { return this.frames[0].prog; }
  get done() { return this.frames.length === 1 && this.frames[0].pc >= this.frames[0].prog.length && !this.run; }
  /** Siguiente paso, resolviendo el fin de subprogramas. */
  next(): Step | null {
    while (this.frames.length) {
      const f = this.frames[this.frames.length - 1];
      if (f.pc < f.prog.length) return f.prog[f.pc++];
      if (this.frames.length === 1) return null;
      this.frames.pop();
    }
    return null;
  }
}

export class RobotActor extends Actor {
  q: number[];
  qd: number[] = [0, 0, 0, 0, 0, 0];
  gripWidth: number = GRIPPER.maxOpen * 0.6;
  /** Velocidad articular máxima (rad/s) y aceleración (rad/s²) al 100 %. */
  vmax = 1.2;
  amax = 2.4;
  /** Velocidad lineal por defecto (mm/s) */
  vlin = 220;
  constructor(id: string, label: string, readonly kin: RobotKin, readonly home: number[], program: Step[]) {
    super(id, label, program);
    this.q = home.slice();
  }
  tcp() { return this.kin.tcpPose(this.q); }
}

export class CustomerActor extends Actor {
  /** Centro de los pies sobre el piso. yaw = 0: mira hacia +Y (hacia la mesa). */
  feet: Vec3;
  yaw = 0;
  /** Recorrido acumulado de la caminata (para balancear las piernas en la vista). */
  stride = 0;
  walking = false;
  present = true;
  handClosed = false;
  /** Mano: posición libre en el mundo, o pegada al cuerpo en reposo / contra el pecho. */
  handMode: 'rest' | 'chest' | 'free' = 'rest';
  private handFree: Vec3 = [0, 0, 0];
  readonly startFeet: Vec3;
  constructor(id: string, label: string, feet: Vec3, program: Step[], startFeet?: Vec3) {
    super(id, label, program);
    this.feet = feet.slice() as Vec3;
    this.startFeet = (startFeet ?? feet).slice() as Vec3;
  }
  private body(local: Vec3): Vec3 {
    const c = Math.cos(this.yaw), s = Math.sin(this.yaw);
    return [this.feet[0] + c * local[0] - s * local[1], this.feet[1] + s * local[0] + c * local[1], this.feet[2] + local[2]];
  }
  /** Hombro derecho (el cliente es diestro: su derecha es +X cuando mira a la mesa). */
  get shoulder(): Vec3 { return this.body([190, 0, 1400]); }
  restHand(): Vec3 { return this.body([230, 60, 840]); }
  chestHand(): Vec3 { return this.body([40, 230, 1140]); }
  get hand(): Vec3 { return this.handMode === 'rest' ? this.restHand() : this.handMode === 'chest' ? this.chestHand() : this.handFree; }
  set hand(v: Vec3) { this.handMode = 'free'; this.handFree = v; }
}

export interface Scenario {
  id: string;
  title: string;
  robots: RobotActor[];
  customer?: CustomerActor;
  objects: SimObject[];
  signals: { id: string; label: string; from: string; to: string }[];
  /** Comprobación del resultado final (para la verificación automática y para el panel). */
  checkResult: (sim: Simulation) => { ok: boolean; text: string };
  /** Mesa / zonas estáticas para colisiones: cajas en el mundo (centro, semiejes). */
  statics: { id: string; c: Vec3; h: Vec3 }[];
  /** Controles de calidad que los programas disparan con el paso `check`. */
  checks?: Record<string, (sim: Simulation) => { ok: boolean; text: string; counter?: string }>;
  /** Producción continua: el ciclo no termina nunca (se repite hasta que se pause). */
  continuous?: boolean;
  /** Poses iniciales de los objetos, para `respawn`. */
  initialPoses?: Record<string, Pose>;
}

export class Simulation {
  time = 0;
  signals = new Map<string, boolean>();
  log: EventEntry[] = [];
  finished = false;
  errors = 0;
  collisions = 0;
  counters = new Map<string, number>();
  logCap = 400;
  /** Distancia mínima robot-robot de todo el ciclo y la actual (mm); la calcula el módulo de colisiones. */
  minClear = Infinity;
  clearNow = Infinity;
  collisionHook: ((sim: Simulation) => void) | null = null;
  private tick = 0;
  private seq = 0;
  private pending = 0;

  constructor(readonly sc: Scenario) {
    for (const s of sc.signals) this.signals.set(s.id, false);
    this.emit('Sistema', `Simulación "${sc.title}" lista`, 'info');
  }

  get actors(): Actor[] {
    return [...this.sc.robots, ...(this.sc.customer ? [this.sc.customer] : [])];
  }
  robot(id: string) { return this.sc.robots.find((r) => r.id === id)!; }
  obj(id: string) { return this.sc.objects.find((o) => o.id === id)!; }

  emit(who: string, text: string, kind: EventEntry['kind'] = 'info') {
    this.log.push({ seq: ++this.seq, t: this.time, who, text, kind });
    if (this.log.length > this.logCap) this.log.shift();
    if (kind === 'error') this.errors++;
    if (who === 'Colisiones') this.collisions++;
  }

  /** Avanza la simulación `seconds` segundos simulados en pasos fijos. */
  advance(seconds: number) {
    // acumulador: el resto fraccionario pasa al siguiente fotograma (exacto a cualquier frecuencia de pantalla)
    this.pending += seconds;
    while (this.pending >= DT && !this.finished) { this.step(); this.pending -= DT; }
  }

  step() {
    for (const a of this.actors) this.stepActor(a, DT);
    this.updateHeldObjects();
    if (this.collisionHook && this.tick % 4 === 0) this.collisionHook(this);
    this.time += DT;
    this.tick++;
    if (!this.sc.continuous && this.actors.every((a) => a.done)) {
      this.finished = true;
      const r = this.sc.checkResult(this);
      this.emit('Sistema', `Ciclo completo en ${this.time.toFixed(1)} s · ${r.text}`, r.ok ? 'info' : 'error');
    }
  }

  // ------------------------------------------------------------------ actores
  private stepActor(a: Actor, dt: number) {
    let budget = dt;
    let guard = 0;
    while (budget > 1e-9 && guard++ < 50) {
      if (!a.run) {
        const step = a.next();
        if (!step) {
          if (a.state !== 'terminado') { a.state = 'terminado'; a.op = 'Ciclo terminado · en reposo'; }
          return;
        }
        if (!this.begin(a, step)) continue; // paso instantáneo
      }
      budget = this.continueRun(a, budget);
    }
  }

  /** Prepara un paso. Devuelve false si fue instantáneo (no consume tiempo). */
  private begin(a: Actor, step: Step): boolean {
    const text = step.k !== 'op' && step.k !== 'if' && 'text' in step && step.text ? step.text : undefined;
    if (text) { a.op = text; this.emit(a.label, text); }
    switch (step.k) {
      case 'op':
        a.op = step.text; this.emit(a.label, step.text);
        return false;
      case 'set':
        if (this.signals.get(step.sig) !== step.value) {
          this.signals.set(step.sig, step.value);
          const def = this.sc.signals.find((s) => s.id === step.sig);
          this.emit(a.label, `${step.value ? '▲' : '▼'} ${def?.label ?? step.sig} = ${step.value ? 1 : 0}`, 'signal');
        }
        return false;
      case 'toggle': {
        const v = !this.signals.get(step.sig);
        this.signals.set(step.sig, v);
        const def = this.sc.signals.find((s) => s.id === step.sig);
        this.emit(a.label, `⇅ ${def?.label ?? step.sig} = ${v ? 1 : 0} (cambio de turno)`, 'signal');
        return false;
      }
      case 'wait':
        a.run = { step, t: 0, dur: step.t }; a.state = 'pausa';
        return true;
      case 'if':
        if (this.signals.get(step.sig) === step.value && (!step.also || (this.signals.get(step.also.a) === this.signals.get(step.also.b)) === step.also.equal)) {
          if (step.text) { a.op = step.text; this.emit(a.label, step.text, 'warn'); }
          a.frames.push({ prog: step.then, pc: 0 });
        }
        return false;
      case 'jump':
        a.frames[a.frames.length - 1].pc = step.to;
        return false;
      case 'respawn': {
        const o = this.obj(step.obj);
        o.holders = [];
        o.pose = this.sc.initialPoses![step.obj];
        if (step.text) this.emit(a.label, step.text);
        return false;
      }
      case 'check': {
        const r = this.sc.checks![step.id](this);
        if (r.ok && r.counter) this.counters.set(r.counter, (this.counters.get(r.counter) ?? 0) + 1);
        this.emit('Calidad', r.text, r.ok ? 'success' : 'error');
        return false;
      }
      case 'present': {
        const c = a as CustomerActor;
        c.present = step.value;
        if (!step.value) { c.feet = c.startFeet.slice() as Vec3; c.yaw = 0; c.handMode = 'rest'; c.handClosed = false; }
        return false;
      }
      case 'walk': {
        const c = a as CustomerActor;
        const d = sub(step.to, c.feet);
        c.run = { step, t: 0, dur: step.t, f0: c.feet.slice() as Vec3, f1: step.to, y0: c.yaw, yDir: Math.atan2(-d[0], d[1]) };
        c.state = 'moviendo';
        c.walking = true;
        return true;
      }
      case 'waitsig':
      case 'waitcmp':
        a.run = { step, t: 0, dur: Infinity }; a.state = 'esperando';
        return true;
      case 'movej': {
        const r = a as RobotActor;
        let q1: number[];
        if (Array.isArray(step.to)) q1 = step.to.slice();
        else {
          const ik = ikNear(r.kin, step.to, r.q);
          if (!ik.ok) {
            this.emit(r.label, `IK sin solución para el destino (error ${ik.posErr.toFixed(1)} mm)`, 'error');
            r.state = 'error';
          }
          q1 = ik.q;
        }
        const sp = step.speed ?? 1;
        let dur = 0.4;
        for (let i = 0; i < 6; i++) {
          const d = Math.abs(q1[i] - r.q[i]);
          dur = Math.max(dur, (MINJERK_VPEAK * d) / (r.vmax * sp), Math.sqrt((MINJERK_APEAK * d) / (r.amax * sp)));
        }
        r.run = { step, t: 0, dur, q0: r.q.slice(), q1 };
        r.state = 'moviendo';
        return true;
      }
      case 'movel': {
        const r = a as RobotActor;
        const p0 = r.tcp();
        const L = norm(sub(step.to.p, p0.p));
        const ang = rotAngle(p0.R, step.to.R);
        const v = step.speed ?? r.vlin;
        const dur = Math.max(0.35, (MINJERK_VPEAK * L) / v, Math.sqrt((MINJERK_APEAK * L) / 600), (MINJERK_VPEAK * ang) / 0.8);
        r.run = { step, t: 0, dur, p0, p1: step.to, q0: r.q.slice() };
        r.state = 'lineal';
        return true;
      }
      case 'grip': {
        const r = a as RobotActor;
        const d = Math.abs(step.width - r.gripWidth);
        r.run = { step, t: 0, dur: Math.max(0.25, d / GRIPPER.speed + 0.15), w0: r.gripWidth };
        r.state = 'pinza';
        // al abrir se suelta lo que tenga sujeto
        if (step.width > r.gripWidth) this.releaseAll(r);
        return true;
      }
      case 'reach': {
        const c = a as CustomerActor;
        c.run = { step, t: 0, dur: step.t, h0: c.hand.slice() as Vec3 };
        c.state = 'moviendo';
        return true;
      }
      case 'hand': {
        const c = a as CustomerActor;
        c.run = { step, t: 0, dur: 0.5 };
        c.state = 'pinza';
        if (!step.close) this.releaseAll(c);
        return true;
      }
    }
  }

  /** Continúa el paso en curso. Devuelve el tiempo sobrante si terminó. */
  private continueRun(a: Actor, budget: number): number {
    const run = a.run!;
    const step = run.step;
    if (step.k === 'waitsig') {
      if (this.signals.get(step.sig) === step.value) { a.run = null; a.state = 'listo'; return budget; }
      return 0;
    }
    if (step.k === 'waitcmp') {
      if ((this.signals.get(step.a) === this.signals.get(step.b)) === step.equal) { a.run = null; a.state = 'listo'; return budget; }
      return 0;
    }
    const used = Math.min(budget, run.dur - run.t);
    run.t += used;
    const s = minJerk(run.t / run.dur);
    switch (step.k) {
      case 'movej': {
        const r = a as RobotActor;
        const qn = run.q0!.map((v, i) => v + (run.q1![i] - v) * s);
        for (let i = 0; i < 6; i++) r.qd[i] = (qn[i] - r.q[i]) / Math.max(used, 1e-6);
        r.q = qn;
        break;
      }
      case 'movel': {
        const r = a as RobotActor;
        const T: Pose = { p: lerp3(run.p0!.p, run.p1!.p, s), R: quatToMat(slerp(matToQuat(run.p0!.R), matToQuat(run.p1!.R), s)) };
        const ik = r.kin.ik(T, r.q, { maxIter: 40 });
        if (!ik.ok && ik.posErr > 1 && !run.warned) {
          run.warned = true;
          this.emit(r.label, `Trayectoria lineal fuera de alcance (${ik.posErr.toFixed(1)} mm)`, 'error');
          r.state = 'error';
        }
        for (let i = 0; i < 6; i++) r.qd[i] = (ik.q[i] - r.q[i]) / Math.max(used, 1e-6);
        r.q = ik.q;
        break;
      }
      case 'grip': {
        const r = a as RobotActor;
        r.gripWidth = run.w0! + (step.width - run.w0!) * s;
        break;
      }
      case 'reach': {
        const c = a as CustomerActor;
        const to = step.to === 'rest' ? c.restHand() : step.to === 'chest' ? c.chestHand() : step.to;
        c.hand = lerp3(run.h0!, to, s);
        if (run.t >= run.dur - 1e-9 && typeof step.to === 'string') c.handMode = step.to;
        break;
      }
      case 'walk': {
        const c = a as CustomerActor;
        const prev = c.feet;
        c.feet = lerp3(run.f0!, run.f1!, s);
        c.stride += norm(sub(c.feet, prev));
        // gira hacia donde camina al empezar y vuelve a mirar la mesa al llegar
        const u = run.t / run.dur;
        const turnIn = Math.min(1, u / 0.18), turnOut = Math.max(0, (u - 0.8) / 0.2);
        const wrap = (x: number) => Math.atan2(Math.sin(x), Math.cos(x));
        const yIn = run.y0! + wrap(run.yDir! - run.y0!) * minJerk(turnIn);
        c.yaw = yIn + wrap(0 - yIn) * minJerk(turnOut);
        if (run.t >= run.dur - 1e-9) { c.walking = false; c.yaw = 0; }
        break;
      }
    }
    if (run.t >= run.dur - 1e-9) {
      a.run = null;
      this.finishStep(a, step);
      return budget - used;
    }
    return 0;
  }

  private finishStep(a: Actor, step: Step) {
    if (a instanceof RobotActor) a.qd = [0, 0, 0, 0, 0, 0];
    if (step.k === 'grip' && step.obj) {
      const r = a as RobotActor;
      if (step.width < r.gripWidth + 1e-6) this.attach(r.id, r.label, r.tcp(), step.obj);
    }
    if (step.k === 'hand' && step.close && step.obj) {
      const c = a as CustomerActor;
      c.handClosed = true;
      this.attach(c.id, c.label, pose(c.hand), step.obj, true);
    }
    if (step.k === 'hand' && !step.close) (a as CustomerActor).handClosed = false;
    if (a.state !== 'error') a.state = 'listo';
  }

  // ------------------------------------------------------------------ agarre
  /** Sujeta el objeto si de verdad está entre las mordazas (o en la mano); si no, es un error de programa. */
  private attach(by: string, label: string, holderPose: Pose, objId: string, keepRotation = false) {
    const o = this.obj(objId);
    const rel = poseMul(poseInv(holderPose), o.pose);
    const d = norm(rel.p);
    const limit = o.kind === 'cube' ? 6 : 70;
    if (d > limit) {
      this.emit(label, `Agarre fallido: ${o.label} a ${d.toFixed(0)} mm de la pinza`, 'error');
      return;
    }
    if (keepRotation) rel.R = mul3(poseInv(holderPose).R, o.pose.R);
    o.holders.push({ by, rel });
    this.emit(label, `Sujeta ${o.label}`);
  }

  private releaseAll(a: Actor) {
    for (const o of this.sc.objects) {
      const i = o.holders.findIndex((h) => h.by === a.id);
      if (i >= 0) {
        o.holders.splice(i, 1);
        this.emit(a.label, o.holders.length ? `Libera ${o.label} (lo sostiene ${this.actorLabel(o.holders[o.holders.length - 1].by)})` : `Deposita ${o.label}`);
      }
    }
  }

  actorLabel(id: string) { return this.actors.find((x) => x.id === id)?.label ?? id; }

  holderPose(id: string): Pose {
    const r = this.sc.robots.find((x) => x.id === id);
    if (r) return r.tcp();
    const c = this.sc.customer!;
    return pose(c.hand);
  }

  private updateHeldObjects() {
    for (const o of this.sc.objects) {
      if (!o.holders.length) continue;
      const h = o.holders[o.holders.length - 1];
      o.pose = poseMul(this.holderPose(h.by), h.rel);
    }
  }
}
