/* The full MCL playground: real-log replay and a drivable simulated robot. */
'use strict';

Widgets.playground = (root, D) => {
  const canvas = root.querySelector('canvas'), read = document.getElementById('play-read'), hint = document.getElementById('play-hint');
  canvas.tabIndex = 0;
  const vp = new UI.Viewport(canvas, D.map, { aspect: 2.3, minH: 300, maxH: 520 });
  const map = D.map, scans = D.scans, box = D.startBox;
  const algo = [...document.querySelectorAll('#algo [data-step]')];
  const flash = steps => algo.forEach(el => el.classList.toggle('hl', steps.includes(el.dataset.step)));

  const S = {
    mode: 'real', running: false, speed: 12, n: 1000, model: 'likelihood', temperature: 1, stride: 5, sigma: 10,
    noise: 1, resampler: 'low_variance', adaptive: true, augmented: false, init: 'box',
    showRays: true, showOdom: true, showRef: false, follow: true, seed: 1,
  };
  let pf, k, est = [], odomPath = [], truthPath = [], hist = { err: [], neff: [], spread: [] }, lastMs = 0;
  // simulation state
  let truth, odo, odoPrev, drOdom, tick, cmd = { v: 0, w: 0 }, keys = {}, autopilot = true, wpI = 1, simRng;
  const WAYPOINTS = [[1822, 30], [1822, 180], [1500, 190], [1000, 200], [620, 190], [900, 170], [1400, 180], [2000, 190], [2600, 180], [2200, 190]];

  const baseAlpha = [0.02, 1e-5, 0.01, 1.0];
  function makeFilter() {
    pf = new MCL.ParticleFilter(map, {
      n: S.n, model: S.model, seed: S.seed++, resampler: S.resampler, threshold: S.adaptive ? 0.5 : 1.01,
      augmented: S.augmented, alpha: baseAlpha.map(a => a * S.noise),
      sensor: { temperature: S.temperature, stride: S.stride, sigma: S.sigma },
    });
    if (S.init === 'global') pf.initGlobal(); else pf.initBox(box.x, box.y, box.theta);
  }
  function reset() {
    makeFilter(); k = 0; est = []; odomPath = []; truthPath = []; hist = { err: [], neff: [], spread: [] };
    if (S.mode === 'sim') {
      simRng = MCL.makeRng(S.seed * 7 + 1);
      truth = [1822, 30, Math.PI / 2]; odo = [0, 0, 0.3]; odoPrev = odo.slice(); drOdom = odo.slice(); tick = 0; wpI = 1;
      truthPath.push(truth.slice());
    }
    if (S.follow) { const b = S.mode === 'sim' ? truth : D.reference.lf[0]; vp.scale = Math.max(vp.scale, vp.h / 520); vp.cx = b[0]; vp.cy = b[1]; }
    flash([]); draw();
  }

  /* --------------------------------------------------------- real replay */
  function stepReal() {
    if (k >= scans.length) { S.running = false; syncPlay(); return; }
    const s = scans[k], t0 = performance.now(), steps = [];
    if (k > 0) { pf.predict(scans[k - 1], s); steps.push('predict'); }
    const applied = pf.update(s[4]);
    if (applied) steps.push('weight', 'neff');
    if (pf.resampled) steps.push('resample');
    lastMs = performance.now() - t0;
    const e = pf.estimate(); est.push(e);
    const r = D.reference.lf[k];
    hist.err.push(Math.hypot(e[0] - r[0], e[1] - r[1])); hist.neff.push(pf.neff); hist.spread.push(pf.spread());
    odomPath.push(D.reference.odom[k]);
    k++; flash(steps);
  }

  /* ----------------------------------------------------------- simulator */
  function autopilotCmd() {
    const wp = WAYPOINTS[wpI], dx = wp[0] - truth[0], dy = wp[1] - truth[1];
    if (Math.hypot(dx, dy) < 12) { wpI = (wpI + 1) % WAYPOINTS.length; if (wpI === 0) wpI = 2; return autopilotCmd(); }
    const e = MCL.wrap(Math.atan2(dy, dx) - truth[2]);
    return { v: Math.abs(e) < 0.35 ? 6 : 0.5, w: Math.max(-0.12, Math.min(0.12, e)) };
  }
  function stepSim() {
    const t0 = performance.now(), steps = [];
    for (let sub = 0; sub < 2; sub++) {
      if (autopilot) cmd = autopilotCmd();
      else cmd = { v: (keys.up ? 6 : 0) - (keys.down ? 4 : 0), w: (keys.left ? 0.1 : 0) - (keys.right ? 0.1 : 0) };
      const nx = truth[0] + cmd.v * Math.cos(truth[2] + cmd.w / 2), ny = truth[1] + cmd.v * Math.sin(truth[2] + cmd.w / 2);
      const prev = truth.slice();
      if (map.isFree(nx, ny) && map.distAt(nx, ny) > 12) { truth[0] = nx; truth[1] = ny; }
      truth[2] = MCL.wrap(truth[2] + cmd.w);
      // odometry: the true increment, corrupted (a little more than the filter assumes) + heading drift
      const [r1, t, r2] = MCL.decompose(prev, truth);
      if (t !== 0 || r2 !== 0) {
        const a = [0.05, 2e-5, 0.02, 2.0];
        const R1 = r1 + Math.sqrt(a[0] * r1 * r1 + a[1] * t * t) * simRng.normal();
        const Tt = t + Math.sqrt(a[2] * t * t + a[3] * (r1 * r1 + r2 * r2)) * simRng.normal();
        const R2 = r2 + Math.sqrt(a[0] * r2 * r2 + a[1] * t * t) * simRng.normal() + 4e-4 * Math.abs(t);
        const h = odo[2] + R1;
        odo = [odo[0] + Tt * Math.cos(h), odo[1] + Tt * Math.sin(h), MCL.wrap(h + R2)];
      }
      tick++;
    }
    truthPath.push(truth.slice());
    const z = MCL.sampleScan(map, truth[0], truth[1], truth[2], simRng);
    pf.predict(odoPrev, odo); steps.push('predict');
    odoPrev = odo.slice();
    if (pf.update(z)) steps.push('weight', 'neff');
    if (pf.resampled) steps.push('resample');
    lastMs = performance.now() - t0;
    const e = pf.estimate(); est.push(e);
    // dead reckoning from the true start
    const o0 = [0, 0, 0.3], s0 = [1822, 30, Math.PI / 2], c = Math.cos(s0[2] - o0[2]), s = Math.sin(s0[2] - o0[2]);
    odomPath.push([s0[0] + c * (odo[0] - o0[0]) - s * (odo[1] - o0[1]), s0[1] + s * (odo[0] - o0[0]) + c * (odo[1] - o0[1])]);
    hist.err.push(Math.hypot(e[0] - truth[0], e[1] - truth[1])); hist.neff.push(pf.neff); hist.spread.push(pf.spread());
    lastScan = z; k++; flash(steps);
  }
  let lastScan = null;

  /* ---------------------------------------------------------------- draw */
  function spark(cv, data, color, fixedMax) {
    const dpr = Math.min(devicePixelRatio || 1, 2), w = cv.clientWidth, h = cv.clientHeight;
    if (cv.width !== Math.round(w * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
    const g = cv.getContext('2d'); g.setTransform(dpr, 0, 0, dpr, 0, 0);
    const T = UI.T(); g.fillStyle = T.surface2; g.fillRect(0, 0, w, h);
    const d = data.slice(-300); if (!d.length) return;
    const mx = fixedMax || Math.max(1, ...d.map(v => (isFinite(v) ? v : 0))) * 1.1;
    g.beginPath(); d.forEach((v, i) => { const x = i / Math.max(1, d.length - 1) * (w - 4) + 2, y = h - 3 - Math.min(v, mx) / mx * (h - 8); i ? g.lineTo(x, y) : g.moveTo(x, y); });
    g.strokeStyle = color; g.lineWidth = 1.8; g.stroke();
    g.fillStyle = T.ink3; g.font = '10px Inter, sans-serif'; g.fillText(mx >= 100 ? mx.toFixed(0) : mx.toFixed(1), 3, 10);
  }
  const charts = document.getElementById('play-charts');
  charts.innerHTML = ['err', 'neff', 'spread'].map(id => `<div><div class="cap"><span id="cap-${id}"></span><b class="mono" id="val-${id}"></b></div><canvas id="spark-${id}"></canvas></div>`).join('');

  function draw() {
    const T = UI.T(), ctx = vp.begin();
    const e = est.length ? est[est.length - 1] : null;
    if (S.follow && !vp.drag) { const f = S.mode === 'sim' ? truth : (e || D.reference.lf[0]); vp.follow(f[0], f[1], 0.12); }
    vp.drawMap();
    if (S.mode === 'real' && S.showRef) vp.path(D.reference.lf, T.s3, 2, [2, 4]);
    if (S.showOdom && odomPath.length) vp.path(odomPath, T.odom, 1.8, [6, 5]);
    if (S.mode === 'sim' && truthPath.length) vp.path(truthPath, T.truth, 2);
    vp.particles(pf.P, pf.n, T.particle, pf.w);
    if (est.length) vp.path(est, T.s1, 2.2);
    if (S.mode === 'sim' && truth) {
      if (S.showRays && lastScan) vp.scan(truth[0], truth[1], truth[2], lastScan, T.truth, 4, true);
      vp.pose(truth[0], truth[1], truth[2], T.truth, 14);
    } else if (S.showRays && e && k > 0) vp.scan(e[0], e[1], e[2], scans[k - 1][4], T.ray, 3, true);
    if (e) vp.pose(e[0], e[1], e[2], T.s1, 14);
    const errLbl = S.mode === 'sim' ? 'error vs truth' : 'distance to Python run';
    const last = a => (a.length ? a[a.length - 1] : NaN);
    read.innerHTML = `${S.mode === 'sim' ? 'scan' : 'scan'} <b>${k}</b>${S.mode === 'real' ? '/' + scans.length : ''} · N<sub>eff</sub> <b>${pf.neff.toFixed(0)}</b>/${pf.n}` +
      ` · ${lastMs.toFixed(1)} ms/update` + (S.augmented && pf.injected ? ` · <span class="pill bad">+${pf.injected} random</span>` : '') +
      (hist.err.length ? `<br>${errLbl}: <b>${last(hist.err).toFixed(1)} cm</b>` : '');
    document.getElementById('cap-err').textContent = S.mode === 'sim' ? 'Error vs ground truth (cm)' : 'Distance to reference run (cm)';
    document.getElementById('cap-neff').textContent = 'N_eff after weighting';
    document.getElementById('cap-spread').textContent = 'Particle spread (cm)';
    document.getElementById('val-err').textContent = hist.err.length ? last(hist.err).toFixed(1) : '';
    document.getElementById('val-neff').textContent = hist.neff.length ? last(hist.neff).toFixed(0) : '';
    document.getElementById('val-spread').textContent = hist.spread.length ? last(hist.spread).toFixed(1) : '';
    spark(document.getElementById('spark-err'), hist.err, T.s2);
    spark(document.getElementById('spark-neff'), hist.neff, T.s1, pf.n);
    spark(document.getElementById('spark-spread'), hist.spread, T.s3);
  }

  /* ------------------------------------------------------------ controls */
  const btn = document.getElementById('play-btn');
  const playBtn = UI.button(btn, '▶ Play', () => { S.running = !S.running; syncPlay(); canvas.focus({ preventScroll: true }); }, 'primary');
  const syncPlay = () => { playBtn.innerHTML = S.running ? '❚❚ Pause' : '▶ Play'; };
  UI.button(btn, 'Step', () => { S.running = false; syncPlay(); (S.mode === 'sim' ? stepSim : stepReal)(); draw(); });
  UI.button(btn, '↺ Reset', () => { reset(); });
  const kidBtn = UI.button(btn, '', () => {
    if (S.mode === 'sim') {
      const r = MCL.makeRng(Date.now() & 0xffff); let x, y;
      do { [x, y] = map.randomFree(r); } while (map.distAt(x, y) < 25 || y < 90 || y > 260);
      truth = [x, y, r.uniform(-Math.PI, Math.PI)]; truthPath.push([NaN, NaN]); truthPath.length = 0; truthPath.push(truth.slice()); wpI = 2;
      hint.textContent = 'Kidnapped! The odometry did not notice. Turn on "Augmented MCL" to let the filter recover.';
    } else { pf.initGlobal(); hint.textContent = 'Particles scattered over the whole map: global localization from here.'; }
    draw();
  });
  UI.button(btn, '＋', () => { vp.zoom(1.4); draw(); });
  UI.button(btn, '－', () => { vp.zoom(1 / 1.4); draw(); });
  UI.button(btn, 'Fit', () => { S.follow = false; followT.set(false); vp.fit(); draw(); });

  const ctl = document.getElementById('play-ctl');
  UI.slider(ctl, { label: 'Particles', min: 50, max: 5000, value: S.n, log: true, fmt: v => Math.round(v), onInput: v => { S.n = Math.round(v); } }).el.addEventListener('change', reset);
  UI.select(ctl, { label: 'Start', value: S.init, options: [['box', 'Known start region (tracking)'], ['global', 'Anywhere (global localization)']], onChange: v => { S.init = v; reset(); } });
  UI.select(ctl, { label: 'Sensor model', value: S.model, options: [['likelihood', 'Likelihood field'], ['beam', 'Beam (ray casting)']], onChange: v => { S.model = v; pf.o.model = v; pf.sensor = Object.assign({}, MCL.SENSOR_DEFAULTS[v], { temperature: S.temperature, stride: S.stride, sigma: S.sigma }); } });
  const live = () => { pf.sensor.temperature = S.temperature; pf.sensor.stride = S.stride; pf.sensor.sigma = S.sigma; pf.o.alpha = baseAlpha.map(a => a * S.noise); };
  UI.slider(ctl, { label: 'Temperature T', min: 1, max: 100, value: S.temperature, log: true, fmt: v => v.toFixed(v < 10 ? 1 : 0), onInput: v => { S.temperature = v; live(); } });
  UI.slider(ctl, { label: 'Beam stride', min: 1, max: 20, step: 1, value: S.stride, fmt: v => `${v} → ${Math.ceil(180 / v)} beams`, onInput: v => { S.stride = v; live(); } });
  UI.slider(ctl, { label: 'σ<sub>hit</sub>', min: 3, max: 60, step: 1, value: S.sigma, fmt: v => v + ' cm', onInput: v => { S.sigma = v; live(); } });
  UI.slider(ctl, { label: 'Motion noise × tuned', min: 0.05, max: 20, value: S.noise, log: true, fmt: v => v.toFixed(2) + '×', onInput: v => { S.noise = v; live(); } });
  UI.select(ctl, { label: 'Resampler', value: S.resampler, options: [['low_variance', 'Low-variance'], ['stratified', 'Stratified'], ['multinomial', 'Multinomial']], onChange: v => { S.resampler = v; pf.o.resampler = v; } });
  UI.toggle(ctl, { label: 'Adaptive resampling (N<sub>eff</sub> &lt; M/2)', value: S.adaptive, onChange: v => { S.adaptive = v; pf.o.threshold = v ? 0.5 : 1.01; } });
  UI.toggle(ctl, { label: 'Augmented MCL (random particles)', value: S.augmented, onChange: v => { S.augmented = v; pf.o.augmented = v; } });
  UI.slider(ctl, { label: 'Speed', min: 1, max: 60, step: 1, value: S.speed, fmt: v => v + ' scans/s', onInput: v => { S.speed = v; } });
  UI.toggle(ctl, { label: 'Show laser rays', value: S.showRays, onChange: v => { S.showRays = v; draw(); } });
  UI.toggle(ctl, { label: 'Show odometry-only path', value: S.showOdom, onChange: v => { S.showOdom = v; draw(); } });
  const refT = UI.toggle(ctl, { label: 'Show Python reference path', value: S.showRef, onChange: v => { S.showRef = v; draw(); } });
  const followT = UI.toggle(ctl, { label: 'Camera follows robot', value: S.follow, onChange: v => { S.follow = v; draw(); } });

  // drive pad
  const drive = document.getElementById('play-drive');
  function buildDrive() {
    drive.innerHTML = '';
    if (S.mode !== 'sim') return;
    const h = document.createElement('h4'); h.textContent = 'Drive'; drive.appendChild(h);
    const ap = UI.toggle(drive, { label: 'Autopilot (hallway tour)', value: autopilot, onChange: v => { autopilot = v; } });
    const pad = document.createElement('div'); pad.className = 'dpad'; pad.style.marginTop = '8px'; drive.appendChild(pad);
    const mk = (lbl, key) => {
      const b = document.createElement('button'); b.className = 'btn'; b.type = 'button'; b.innerHTML = lbl; b.setAttribute('aria-label', key);
      const on = e => { e.preventDefault(); keys[key] = true; autopilot = false; ap.set(false); if (!S.running) { S.running = true; syncPlay(); } };
      const off = () => { keys[key] = false; };
      b.addEventListener('pointerdown', on); b.addEventListener('pointerup', off); b.addEventListener('pointerleave', off); b.addEventListener('pointercancel', off);
      return b;
    };
    const blank = () => document.createElement('span');
    pad.append(blank(), mk('▲', 'up'), blank(), mk('◀', 'left'), mk('▼', 'down'), mk('▶', 'right'));
    const p = document.createElement('p'); p.className = 'small muted'; p.style.margin = '8px 0 0'; p.innerHTML = 'or click the map and use the arrow keys'; drive.appendChild(p);
  }
  const KEYMAP = { ArrowUp: 'up', ArrowDown: 'down', ArrowLeft: 'left', ArrowRight: 'right', w: 'up', s: 'down', a: 'left', d: 'right' };
  canvas.addEventListener('keydown', e => {
    if (S.mode !== 'sim' || !KEYMAP[e.key]) return;
    e.preventDefault(); keys[KEYMAP[e.key]] = true; autopilot = false; buildDrive(); if (!S.running) { S.running = true; syncPlay(); }
  });
  canvas.addEventListener('keyup', e => { if (KEYMAP[e.key]) keys[KEYMAP[e.key]] = false; });

  // mode switch
  const legend = document.getElementById('play-legend');
  const setMode = m => {
    S.mode = m; S.running = false; syncPlay();
    root.querySelectorAll('#play-mode button').forEach(b => b.classList.toggle('on', b.dataset.mode === m));
    kidBtn.innerHTML = m === 'sim' ? '⚡ Kidnap robot' : '⚡ Scatter particles';
    refT.el.closest('label').style.display = m === 'real' ? '' : 'none';
    hint.textContent = m === 'sim' ? 'Green is the true robot (unknown to the filter). The laser scan is simulated from the real map.' : 'Replaying the real 45 s log. No ground truth: compare to the Python run and to odometry alone.';
    legend.innerHTML = `<span><i style="background:var(--particle)"></i>particles (size ∝ weight)</span><span><i class="line" style="border-color:var(--series-1)"></i>MCL estimate</span>` +
      (m === 'sim' ? `<span><i class="line" style="border-color:var(--truth)"></i>true path</span>` : `<span><i class="dash" style="border-color:var(--series-3)"></i>Python reference</span>`) +
      `<span><i class="dash" style="border-color:var(--odom)"></i>odometry only</span>`;
    buildDrive(); reset();
  };
  root.querySelectorAll('#play-mode button').forEach(b => b.addEventListener('click', () => setMode(b.dataset.mode)));

  /* ---------------------------------------------------------------- loop */
  let visible = false, acc = 0, lastT = performance.now();
  UI.whenVisible(root, () => { visible = true; }, () => { visible = false; });
  const loop = now => {
    const dt = Math.min(0.1, (now - lastT) / 1000); lastT = now;
    if (visible) {
      if (S.running) {
        acc += dt * S.speed;
        let n = 0;
        while (acc >= 1 && n < 6) { (S.mode === 'sim' ? stepSim : stepReal)(); acc -= 1; n++; }
        if (acc > 6) acc = 0;
      }
      draw();
    }
    requestAnimationFrame(loop);
  };
  vp.onChange = draw; UI.onTheme(() => { vp.mapImg = null; draw(); });
  setMode('real');
  requestAnimationFrame(loop);
};
