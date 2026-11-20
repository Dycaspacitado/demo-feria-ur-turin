/**
 * Interfaz: selección de simulación, reproducción, velocidad, cámaras y paneles de estado.
 */
import { Simulation, type RobotActor, type Scenario, type ActorState } from '../sim/engine';
import { installCollision } from '../sim/collision';
import { buildKeychainScenario } from '../sim/scenarioKeychain';
import { buildTowerScenario } from '../sim/scenarioTower';
import { buildServiceScenario } from '../sim/scenarioService';
import { CAMERA_PRESETS, type CameraPreset, Scene3D } from '../view/scene3d';

const SIMS = [
  { id: 'llavero', short: '1 · Entrega de llavero', build: buildKeychainScenario },
  { id: 'torre', short: '2 · Torre GRUPRO', build: buildTowerScenario },
  { id: 'servicio', short: '3 · Producción + visitante', build: buildServiceScenario },
] as const;

const SPEEDS = [0.25, 0.5, 1, 1.5, 2, 3, 4];
const STATE_TXT: Record<ActorState, string> = {
  listo: 'Listo', moviendo: 'Mov. articular', lineal: 'Mov. lineal', pinza: 'Pinza', esperando: 'Esperando señal',
  pausa: 'Pausa', terminado: 'Terminado', error: 'Error',
};

const h = <K extends keyof HTMLElementTagNameMap>(tag: K, cls = '', html = ''): HTMLElementTagNameMap[K] => {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (html) e.innerHTML = html;
  return e;
};
const deg = (r: number) => (r * 180) / Math.PI;

export class App {
  private view: Scene3D;
  private sim!: Simulation;
  private simIdx = 0;
  private playing = false;
  private speed = 1;
  private repeat = false;
  private cycleTotal = 1;
  private restartAt = -1;
  private lastUi = 0;
  private lastFrame = performance.now();
  private logCount = 0;
  private els: Record<string, HTMLElement> = {};
  private bannerUntil = 0;

  constructor(root: HTMLElement) {
    root.innerHTML = '';
    const header = h('header', 'top');
    header.append(
      h('div', 'brand', '<span class="logo">GRUPRO</span><span class="title">Simulación de cobots · UR5 + Turin TCR050</span>'),
    );
    const tabs = h('nav', 'tabs');
    SIMS.forEach((s, i) => {
      const b = h('button', 'tab', s.short);
      b.onclick = () => this.select(i);
      tabs.append(b);
    });
    header.append(tabs);
    this.els.tabs = tabs;

    const main = h('main', 'main');
    const vp = h('div', 'viewport');
    const panel = h('aside', 'panel');
    main.append(vp, panel);
    root.append(header, main);
    this.view = new Scene3D(vp);

    // superposiciones del visor
    const chip = h('div', 'chip');
    this.els.chip = chip;
    const cams = h('div', 'cams');
    (Object.keys(CAMERA_PRESETS) as CameraPreset[]).forEach((k) => {
      const b = h('button', '', CAMERA_PRESETS[k].label);
      b.onclick = () => this.view.setCamera(k);
      cams.append(b);
    });
    const bar = h('div', 'progress', '<div class="fill"></div><span class="ptxt"></span>');
    this.els.fill = bar.querySelector('.fill')!;
    this.els.ptxt = bar.querySelector('.ptxt')!;
    const banner = h('div', 'banner');
    this.els.banner = banner;
    vp.append(chip, cams, bar, banner);

    // panel lateral
    const ctl = h('section', 'card controls');
    ctl.innerHTML = `
      <div class="row">
        <button class="primary" data-a="play">▶ Iniciar</button>
        <button data-a="reset">↺ Reiniciar</button>
      </div>
      <label class="speed">Velocidad <input type="range" min="0" max="${SPEEDS.length - 1}" step="1" value="2"><b>1×</b></label>
      <button class="call" data-a="call">🔔 Llamar a un visitante</button>
      <div class="checks">
        <label class="repeat"><input type="checkbox" data-c="repeat"> Repetir ciclo (modo feria)</label>
        <label><input type="checkbox" data-c="trails" checked> Trayectorias del TCP</label>
        <label><input type="checkbox" data-c="env"> Envolventes de colisión</label>
      </div>`;
    this.els.play = ctl.querySelector('[data-a=play]')!;
    this.els.play.onclick = () => this.toggle();
    (ctl.querySelector('[data-a=reset]') as HTMLElement).onclick = () => this.reset();
    this.els.call = ctl.querySelector('[data-a=call]')!;
    this.els.call.onclick = () => this.callVisitor();
    this.els.repeatRow = ctl.querySelector('.repeat')!;
    const range = ctl.querySelector('input[type=range]') as HTMLInputElement;
    const sp = ctl.querySelector('.speed b')!;
    range.oninput = () => { this.speed = SPEEDS[+range.value]; sp.textContent = `${this.speed}×`; };
    (ctl.querySelector('[data-c=repeat]') as HTMLInputElement).onchange = (e) => (this.repeat = (e.target as HTMLInputElement).checked);
    (ctl.querySelector('[data-c=trails]') as HTMLInputElement).onchange = (e) => (this.view.showTrails = (e.target as HTMLInputElement).checked);
    (ctl.querySelector('[data-c=env]') as HTMLInputElement).onchange = (e) => (this.view.showEnvelopes = (e.target as HTMLInputElement).checked);

    this.els.robots = h('div', 'robots');
    const sig = h('section', 'card', '<h3>Señales de coordinación (E/S digitales)</h3><div class="signals"></div>');
    this.els.signals = sig.querySelector('.signals')!;
    const safe = h('section', 'card', '<h3>Seguridad y colisiones</h3><div class="safety"></div>');
    this.els.safety = safe.querySelector('.safety')!;
    const log = h('section', 'card logcard', '<h3>Registro de eventos</h3><div class="log"></div>');
    this.els.log = log.querySelector('.log')!;
    panel.append(ctl, this.els.robots, sig, safe, log);

    window.addEventListener('keydown', (e) => {
      if ((e.target as HTMLElement).tagName === 'INPUT') return;
      if (e.code === 'Space') { e.preventDefault(); this.toggle(); }
      if (e.key === 'r' || e.key === 'R') this.reset();
      if (e.key === '1') this.select(0);
      if (e.key === '2') this.select(1);
      if (e.key === '3') this.select(2);
      if (e.key === 'v' || e.key === 'V') this.callVisitor();
    });

    this.select(0);
    requestAnimationFrame(this.frame);
  }

  private select(i: number) {
    this.simIdx = i;
    [...this.els.tabs.children].forEach((b, k) => b.classList.toggle('active', k === i));
    // duración total del ciclo: se ejecuta una vez sin interfaz (rápido) para la barra de progreso
    const sc = SIMS[i].build();
    if (sc.continuous) this.cycleTotal = Infinity;
    else {
      const probe = new Simulation(sc);
      while (!probe.finished && probe.time < 400) probe.step();
      this.cycleTotal = probe.time;
    }
    this.els.call.style.display = sc.continuous ? '' : 'none';
    this.els.repeatRow.style.display = sc.continuous ? 'none' : '';
    this.reset(true);
    this.view.setCamera('general', false);
  }

  private reset(keepPaused = false) {
    const sc: Scenario = SIMS[this.simIdx].build();
    this.sim = new Simulation(sc);
    installCollision(this.sim);
    this.view.load(sc);
    this.view.resetTrails();
    this.restartAt = -1;
    this.logCount = 0;
    this.els.log.innerHTML = '';
    this.buildRobotCards(sc);
    this.buildSignals(sc);
    this.els.banner.className = 'banner';
    this.setPlaying(keepPaused ? false : this.playing);
    this.updateUi(true);
  }

  private toggle() {
    if (this.sim.finished) { this.reset(); this.setPlaying(true); return; }
    this.setPlaying(!this.playing);
  }

  private setPlaying(p: boolean) {
    this.playing = p;
    this.els.play.textContent = p ? '⏸ Pausar' : this.sim.time > 0 && !this.sim.finished ? '▶ Reanudar' : '▶ Iniciar';
    this.els.play.classList.toggle('primary', !p);
  }

  private frame = (now: number) => {
    const dt = Math.min(0.1, (now - this.lastFrame) / 1000);
    this.lastFrame = now;
    if (this.playing && !this.sim.finished) {
      this.sim.advance(dt * this.speed);
      if (this.sim.finished) this.onFinished();
    }
    if (this.restartAt > 0 && now >= this.restartAt) {
      this.restartAt = -1;
      this.reset();
      this.setPlaying(true);
    }
    this.view.sync(this.sim, dt);
    if (now - this.lastUi > 90) { this.lastUi = now; this.updateUi(); }
    requestAnimationFrame(this.frame);
  };

  /** Pide una visita (señal LLAMADA): el visitante entra, pulsa el botón y los robots lo atienden. */
  private callVisitor() {
    const sim = this.sim;
    if (!sim.sc.continuous || !sim.sc.customer) return;
    if (sim.signals.get('LLAMADA') || sim.sc.customer.present) {
      this.showBanner('Ya hay un visitante siendo atendido', 'bad');
      return;
    }
    sim.signals.set('LLAMADA', true);
    sim.emit('Interfaz', '▲ LLAMADA = 1 (se pidió un visitante)', 'signal');
    if (!this.playing) this.setPlaying(true);
  }

  private showBanner(text: string, kind: 'ok' | 'bad') {
    this.els.banner.innerHTML = text;
    this.els.banner.className = `banner show ${kind}`;
    this.bannerUntil = performance.now() + 3500;
  }

  private onFinished() {
    this.setPlaying(false);
    const r = this.sim.sc.checkResult(this.sim);
    this.els.banner.innerHTML = `${r.ok ? '✔' : '✖'} ${r.text}`;
    this.els.banner.className = `banner show ${r.ok ? 'ok' : 'bad'}`;
    if (this.repeat) this.restartAt = performance.now() + 3500;
  }

  // ------------------------------------------------------------------ paneles
  private buildRobotCards(sc: Scenario) {
    this.els.robots.innerHTML = '';
    for (const r of sc.robots) {
      const c = h('section', `card robot ${r.id}`);
      c.innerHTML = `
        <div class="rhead"><h3>${r.label}</h3><span class="state"></span></div>
        <div class="op"></div>
        <div class="grip"></div>
        <div class="joints">${[1, 2, 3, 4, 5, 6].map((j) => `<div class="j"><span>J${j}</span><div class="track"><div class="knob"></div></div><b></b></div>`).join('')}</div>
        <div class="tcp"></div>`;
      c.dataset.id = r.id;
      this.els.robots.append(c);
    }
    if (sc.customer) {
      const c = h('section', 'card robot cliente');
      c.dataset.id = 'cliente';
      c.innerHTML = `<div class="rhead"><h3>${sc.customer.label}</h3><span class="state"></span></div><div class="op"></div>`;
      this.els.robots.append(c);
    }
  }

  private buildSignals(sc: Scenario) {
    this.els.signals.innerHTML = sc.signals
      .map((s) => `<div class="sig" data-s="${s.id}"><i></i><b>${s.label}</b><span>${s.from} → ${s.to}</span></div>`)
      .join('');
  }

  private updateUi(force = false) {
    const sim = this.sim;
    const sc = sim.sc;
    for (const card of this.els.robots.children as HTMLCollectionOf<HTMLElement>) {
      const id = card.dataset.id!;
      const a = sim.actors.find((x) => x.id === id)!;
      const st = card.querySelector('.state') as HTMLElement;
      st.textContent = STATE_TXT[a.state];
      st.className = `state s-${a.state}`;
      (card.querySelector('.op') as HTMLElement).textContent = a.op;
      if (id === 'cliente') continue;
      const r = a as RobotActor;
      const held = sc.objects.filter((o) => o.holders.some((x) => x.by === r.id)).map((o) => o.label);
      (card.querySelector('.grip') as HTMLElement).innerHTML =
        `<span class="pill ${held.length ? 'closed' : ''}">Pinza ${held.length ? 'cerrada' : 'abierta'} · ${r.gripWidth.toFixed(0)} mm</span>${held.length ? `<span class="held">sujeta ${held.join(', ')}</span>` : ''}`;
      const js = card.querySelectorAll('.j');
      r.q.forEach((q, i) => {
        const lim = r.kin.limits[i];
        const f = (q - lim[0]) / (lim[1] - lim[0]);
        (js[i].querySelector('.knob') as HTMLElement).style.left = `${(f * 100).toFixed(1)}%`;
        (js[i].querySelector('b') as HTMLElement).textContent = `${deg(q).toFixed(1)}°`;
      });
      const p = r.tcp().p;
      (card.querySelector('.tcp') as HTMLElement).textContent = `TCP  X ${p[0].toFixed(0)}  Y ${p[1].toFixed(0)}  Z ${p[2].toFixed(0)} mm`;
    }
    for (const row of this.els.signals.children as HTMLCollectionOf<HTMLElement>) row.classList.toggle('on', !!sim.signals.get(row.dataset.s!));

    const cl = sim.clearNow, mn = sim.minClear;
    const collisions = sim.collisions;
    this.els.safety.innerHTML = `
      <div class="kv"><span>Distancia actual UR5 ↔ Turin</span><b class="${cl < 30 ? 'warn' : ''}">${isFinite(cl) ? cl.toFixed(0) + ' mm' : '—'}</b></div>
      <div class="kv"><span>Mínima en el ciclo</span><b>${isFinite(mn) ? mn.toFixed(0) + ' mm' : '—'}</b></div>
      <div class="kv"><span>Contactos indebidos</span><b class="${collisions ? 'bad' : 'good'}">${collisions ? collisions : 'ninguno'}</b></div>
      <p class="note">Distancias medidas entre envolventes esféricas de la malla original (conservadoras): la distancia real es mayor.</p>`;

    // registro (solo lo nuevo)
    const fresh = sim.log.filter((e) => e.seq > this.logCount);
    if (fresh.length || force) {
      for (const e of fresh) {
        if (e.kind === 'success' && !force) this.showBanner(`✔ ${e.text}`, 'ok');
        const row = h('div', `ev ${e.kind}`, `<time>${e.t.toFixed(1)}s</time><b>${e.who}</b><span></span>`);
        (row.querySelector('span') as HTMLElement).textContent = e.text;
        this.els.log.prepend(row);
      }
      while (this.els.log.children.length > 160) this.els.log.lastChild!.remove();
      if (fresh.length) this.logCount = fresh[fresh.length - 1].seq;
    }

    if (this.bannerUntil && performance.now() > this.bannerUntil && !sim.finished) {
      this.bannerUntil = 0;
      this.els.banner.className = 'banner';
    }
    const cont = !isFinite(this.cycleTotal);
    const f = cont ? 1 : Math.min(1, sim.time / this.cycleTotal);
    this.els.fill.style.width = `${(f * 100).toFixed(1)}%`;
    this.els.ptxt.textContent = cont
      ? `${sim.time.toFixed(0)} s · torres armadas: ${sim.counters.get('torres') ?? 0} · ciclos completos: ${sim.counters.get('ciclos') ?? 0} · llaveros entregados: ${sim.counters.get('llaveros') ?? 0}`
      : `${sim.time.toFixed(1)} s / ${this.cycleTotal.toFixed(1)} s`;
    this.els.fill.classList.toggle('cont', cont);
    const busy = !!sim.signals.get('LLAMADA') || !!sim.sc.customer?.present;
    (this.els.call as HTMLButtonElement).disabled = busy;
    this.els.call.textContent = busy ? (sim.signals.get('PEDIDO') ? '⏳ Atendiendo el pedido…' : '🚶 Visitante en el stand…') : '🔔 Llamar a un visitante';
    const status = sim.finished ? 'Ciclo completo' : this.playing ? 'En ejecución' : sim.time > 0 ? 'En pausa' : 'Listo para iniciar';
    this.els.chip.innerHTML = `<b>${sc.title}</b><span class="dot ${this.playing ? 'run' : ''}"></span>${status} · ${this.speed}×`;
  }
}
