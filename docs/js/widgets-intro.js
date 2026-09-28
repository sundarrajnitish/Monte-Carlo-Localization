/* Hero, "localize it yourself", and the 1-D corridor. */
'use strict';

const Widgets = {};

/* ------------------------------------------------------------------ helpers */
Widgets.frameFor = (vp, map) => {
  // fit the whole hallway if it is readable, otherwise zoom in and follow
  const s = Math.min(vp.w / map.width, vp.h / map.height);
  vp.fit();
  vp.narrow = vp.w / map.width < 0.3;
  if (vp.narrow) vp.scale = Math.max(s, vp.h / 420);
};

/* =================================================================== HERO */
Widgets.hero = (root, D) => {
  const canvas = root.querySelector('canvas'), read = document.getElementById('hero-read');
  const vp = new UI.Viewport(canvas, D.map, { aspect: 8.0, minH: window.innerWidth < 600 ? 220 : 150, maxH: 360, pan: false });
  Widgets.frameFor(vp, D.map);
  let pf, k, running = true, visible = false, hold = 0, seed = 7;
  const scans = D.scans;
  const reset = () => {
    pf = new MCL.ParticleFilter(D.map, { n: 3000, model: 'likelihood', sensor: { temperature: 30 }, seed: seed++ });
    pf.initGlobal(); k = 0; hold = 0;
  };
  reset();
  const ctl = document.getElementById('hero-ctl');
  const playBtn = UI.button(ctl, '❚❚ Pause', () => { running = !running; playBtn.innerHTML = running ? '❚❚ Pause' : '▶ Play'; });
  UI.button(ctl, '↺ Restart with new random guesses', () => { reset(); draw(); });
  const step = () => {
    if (k >= scans.length) { if (++hold > 90) reset(); return; }
    const s = scans[k];
    if (k > 0) pf.predict(scans[k - 1], s);
    pf.update(s[4]);
    k++;
  };
  const draw = () => {
    const T = UI.T(); vp.begin(); vp.drawMap();
    vp.particles(pf.P, pf.n, T.particle);
    const e = pf.estimate(), conv = pf.spread() < 60;
    if (vp.narrow) vp.follow(e[0], e[1], conv ? 0.2 : 0.02);
    if (k > 0 && conv) vp.scan(e[0], e[1], e[2], scans[k - 1][4], T.ray, 3, true);
    if (conv) vp.pose(e[0], e[1], e[2], T.s1, 14);
    read.innerHTML = `scan <b>${k}</b>/${scans.length} · N<sub>eff</sub> <b>${pf.neff.toFixed(0)}</b> · spread <b>${pf.spread().toFixed(0)} cm</b>` +
      (conv ? ' · <span class="pill good">converged</span>' : ' · <span class="pill">searching…</span>');
  };
  let frame = 0;
  const loop = () => { if (visible) { if (running && frame++ % 3 === 0) step(); draw(); } requestAnimationFrame(loop); };
  UI.whenVisible(root, () => { visible = true; }, () => { visible = false; });
  vp.onChange = draw; UI.onTheme(() => { vp.mapImg = null; draw(); });
  requestAnimationFrame(loop);

  const g = D.results.real_global.find(r => r.n === 3000 && r.temperature === 30);
  if (g) document.getElementById('hero-rate').innerHTML = `With these settings it finds the right place in ${g.success} of ${g.runs} runs; <a href="#findings">chapter 7</a> explains the rest.`;
  const R = D.results.real_tracking_summary;
  document.getElementById('hero-stats').innerHTML = [
    [`${(D.map.width / 100).toFixed(0)} × ${(D.map.height / 100).toFixed(1)} m`, 'hallway map, 4 cm cells'],
    [`${scans.length}`, `laser scans over ${R.duration_s.toFixed(0)} s, 180 beams each`],
    [`${R.median_residual_cm.likelihood_field.toFixed(0)} cm`, `median scan-to-wall error with MCL (odometry alone: ${R.median_residual_cm.dead_reckoning.toFixed(0)} cm)`],
    [`${R.ms_per_scan.likelihood_field.toFixed(1)} ms`, 'per scan, 1000 particles, likelihood field (Python)'],
  ].map(([b, s]) => `<div class="stat"><b>${b}</b><span>${s}</span></div>`).join('');
};

/* ============================================================ MANUAL */
Widgets.manual = (root, D) => {
  const canvas = root.querySelector('canvas'), out = document.getElementById('manual-score');
  const vp = new UI.Viewport(canvas, D.map, { aspect: 3.2, minH: 260, maxH: 460 });
  const ref = D.reference.lf;
  let k = 120, heat = null;
  const pose = [1400, 150, 0.4];
  vp.fit([pose[0] - 700, -20, pose[0] + 700, D.map.height + 20]);
  const sens = Object.assign({}, MCL.SENSOR_DEFAULTS.likelihood, { stride: 2 });
  vp.addHandle({ get: () => [pose[0], pose[1]], set: (x, y) => { pose[0] = x; pose[1] = y; draw(); }, r: 22 });
  vp.addHandle({
    get: () => [pose[0] + 45 * Math.cos(pose[2]), pose[1] + 45 * Math.sin(pose[2])],
    set: (x, y) => { pose[2] = Math.atan2(y - pose[1], x - pose[0]); draw(); }, r: 16,
  });
  const ctl = document.getElementById('manual-ctl');
  const scanSl = UI.slider(ctl, { label: 'Scan from the log', min: 0, max: D.scans.length - 1, step: 1, value: k, fmt: v => `#${v}`, onInput: v => { k = v; heat = null; draw(); } });
  const btn = document.getElementById('manual-btn');
  let answer = false;
  UI.button(btn, 'Show the filter\'s answer', () => { answer = !answer; draw(); });
  UI.button(btn, 'Heat map', () => { heat = heat ? null : computeHeat(); draw(); });
  UI.button(btn, 'Random pose', () => { const [x, y] = D.map.randomFree(MCL.makeRng(Date.now() & 0xffff)); pose[0] = x; pose[1] = y; pose[2] = Math.random() * 6.28 - 3.14; draw(); });
  void scanSl;

  function score(x, y, th) {
    if (!D.map.isFree(x, y)) return -Infinity;
    return MCL.logLikelihood('likelihood', sens, D.map, x, y, th, D.scans[k][4])[0];
  }
  function computeHeat() {
    const step = 20, W = Math.ceil(D.map.width / step), H = Math.ceil(D.map.height / step);
    const g = new Float32Array(W * H).fill(-Infinity);
    const s5 = Object.assign({}, sens, { stride: 6 });
    let best = -Infinity;
    for (let j = 0; j < H; j++) for (let i = 0; i < W; i++) {
      const x = (i + 0.5) * step, y = (j + 0.5) * step;
      if (!D.map.isFree(x, y)) continue;
      let b = -Infinity;
      for (let a = 0; a < 24; a++) b = Math.max(b, MCL.logLikelihood('likelihood', s5, D.map, x, y, -Math.PI + a * Math.PI / 12, D.scans[k][4])[0]);
      g[j * W + i] = b; best = Math.max(best, b);
    }
    const off = document.createElement('canvas'); off.width = W; off.height = H;
    const cx = off.getContext('2d'), img = cx.createImageData(W, H);
    const col = UI.T().s1, v = parseInt(col.slice(1), 16), rgb = [v >> 16 & 255, v >> 8 & 255, v & 255];
    for (let j = 0; j < H; j++) for (let i = 0; i < W; i++) {
      const val = g[j * W + i], p = ((H - 1 - j) * W + i) * 4;
      const a = isFinite(val) ? Math.exp((val - best) / 8) : 0;       // softened posterior
      img.data[p] = rgb[0]; img.data[p + 1] = rgb[1]; img.data[p + 2] = rgb[2]; img.data[p + 3] = Math.round(230 * a);
    }
    cx.putImageData(img, 0, 0);
    return off;
  }
  function draw() {
    const T = UI.T(); vp.begin(); vp.drawMap();
    if (heat) { const [x0, y0] = vp.toScreen(0, D.map.height); vp.ctx.imageSmoothingEnabled = true; vp.ctx.drawImage(heat, x0, y0, D.map.width * vp.scale, D.map.height * vp.scale); }
    const z = D.scans[k][4], th = pose[2];
    const lx = pose[0] + MCL.LASER_OFFSET * Math.cos(th), ly = pose[1] + MCL.LASER_OFFSET * Math.sin(th);
    const ctx = vp.ctx;
    let near = 0, tot = 0;
    for (let b = 0; b < 180; b += 2) {
      if (z[b] >= MCL.MAX_RANGE - 1) continue;
      const a = th + MCL.BEARINGS[b], ex = lx + z[b] * Math.cos(a), ey = ly + z[b] * Math.sin(a);
      const d = D.map.distAt(ex, ey), good = d < 15;
      near += good; tot++;
      const [sx, sy] = vp.toScreen(ex, ey), [ox, oy] = vp.toScreen(lx, ly);
      ctx.strokeStyle = T.ray; ctx.globalAlpha = 0.15; ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(sx, sy); ctx.stroke(); ctx.globalAlpha = 1;
      ctx.fillStyle = good ? T.truth : T.particle; ctx.beginPath(); ctx.arc(sx, sy, 3, 0, 7); ctx.fill();
    }
    if (answer) { const r = ref[k]; vp.pose(r[0], r[1], r[2], T.s1, 14, false); }
    vp.pose(pose[0], pose[1], pose[2], T.ink3, 16);
    const [hx, hy] = vp.toScreen(pose[0] + 45 * Math.cos(pose[2]), pose[1] + 45 * Math.sin(pose[2]));
    ctx.fillStyle = T.ink; ctx.beginPath(); ctx.arc(hx, hy, 6, 0, 7); ctx.fill();
    const s = score(...pose);
    const r = ref[k], dist = Math.hypot(r[0] - pose[0], r[1] - pose[1]);
    out.innerHTML = `log-likelihood <b>${isFinite(s) ? s.toFixed(1) : '−∞ (in a wall)'}</b><br>${near}/${tot} end-points on a wall` +
      (answer ? `<br>filter says (${r[0].toFixed(0)}, ${r[1].toFixed(0)}) · you are ${dist.toFixed(0)} cm off` : '');
  }
  vp.onChange = draw; UI.onTheme(() => { vp.mapImg = null; heat = null; draw(); });
  draw();
};

/* ========================================================== 1-D CORRIDOR */
Widgets.corridor = (root) => {
  const canvas = root.querySelector('canvas'), ctx = canvas.getContext('2d');
  const status = document.getElementById('w1d-status');
  const L = 15, XMAX = 20, DOORS = [3, 5, 10], SIG = 0.2;
  const pDoor = x => 0.05 + 0.9 * DOORS.reduce((s, d) => s + Math.exp(-((x - d) ** 2) / (2 * SIG * SIG)), 0);
  let M = 500, rng = MCL.makeRng(3), x = new Float64Array(M), w = new Float64Array(M), truth = 1.2, last = '', scheme = 'low_variance', a3 = 0.01;
  let neffBefore = M, auto = null, phase = 0;
  const reset = () => { x = Float64Array.from({ length: M }, () => rng.rand() * L); w = new Float64Array(M).fill(1 / M); truth = 0.5 + rng.rand() * 1.5; last = 'Particles spread uniformly over the corridor.'; neffBefore = M; draw(); };
  const move = (u = 1) => {
    for (let i = 0; i < M; i++) x[i] += u + Math.sqrt(a3 * Math.abs(u)) * rng.normal();
    truth += u + Math.sqrt(a3 * Math.abs(u)) * rng.normal() * 0.5;
    if (truth > L - 0.3) truth -= L - 1;                   // wrap the robot back for endless play
    last = `Moved ${u} m: every particle moved too, plus noise. Belief spreads out.`; draw();
  };
  const sense = () => {
    const z = rng.rand() < pDoor(truth) ? 1 : 0;
    let s = 0;
    for (let i = 0; i < M; i++) { w[i] = x[i] > L || x[i] < 0 ? 0 : (z ? pDoor(x[i]) : 1 - pDoor(x[i])); s += w[i]; }
    for (let i = 0; i < M; i++) w[i] /= s;
    neffBefore = 1 / w.reduce((a, b) => a + b * b, 0);
    const idx = MCL.RESAMPLERS[scheme](w, M, rng);
    const nx = new Float64Array(M), uniq = new Set(idx);
    for (let m = 0; m < M; m++) nx[m] = x[idx[m]];
    x = nx; w.fill(1 / M);
    last = `Sensed <b>${z ? 'a door' : 'no door'}</b> → weighted (N<sub>eff</sub> ${neffBefore.toFixed(0)}/${M}) → resampled: ${uniq.size} distinct particles survived.`;
    draw();
  };
  const btn = document.getElementById('w1d-btn');
  UI.button(btn, 'Move 1 m', () => move(1), 'primary');
  UI.button(btn, 'Sense', sense, 'primary');
  const autoBtn = UI.button(btn, '▶ Auto', () => {
    if (auto) { clearInterval(auto); auto = null; autoBtn.innerHTML = '▶ Auto'; return; }
    autoBtn.innerHTML = '❚❚ Stop';
    auto = setInterval(() => { (phase++ % 2 ? sense : move)(); }, 650);
  });
  UI.button(btn, '↺ Reset', reset);
  const ctl = document.getElementById('w1d-ctl');
  UI.select(ctl, { label: 'Resampler', value: scheme, options: [['low_variance', 'Low-variance'], ['multinomial', 'Multinomial'], ['stratified', 'Stratified']], onChange: v => { scheme = v; } });
  UI.slider(ctl, { label: 'Motion noise α₃', min: 0.001, max: 0.2, value: a3, log: true, fmt: v => v.toFixed(3), onInput: v => { a3 = v; } });
  UI.slider(ctl, { label: 'Particles M', min: 50, max: 2000, step: 50, value: M, onInput: v => { M = v; reset(); } });

  function resize() {
    const dpr = Math.min(devicePixelRatio || 1, 2), wpx = canvas.parentElement.clientWidth, h = Math.max(260, Math.min(340, wpx * 0.38));
    canvas.width = wpx * dpr; canvas.height = h * dpr; canvas.style.width = wpx + 'px'; canvas.style.height = h + 'px';
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0); canvas._w = wpx; canvas._h = h;
  }
  function draw() {
    const T = UI.T(), W = canvas._w, H = canvas._h, pad = 20, sx = v => pad + v / XMAX * (W - 2 * pad);
    ctx.fillStyle = T.surface2; ctx.fillRect(0, 0, W, H);
    // corridor with doors
    const cy = 42;
    ctx.fillStyle = T.wall; ctx.fillRect(sx(0), cy - 16, sx(L) - sx(0), 10);
    DOORS.forEach(d => { ctx.fillStyle = T.floor; ctx.fillRect(sx(d - 0.35), cy - 16, sx(0.7) - sx(0), 10); ctx.strokeStyle = T.wall; ctx.strokeRect(sx(d - 0.35), cy - 30, sx(0.7) - sx(0), 14); });
    ctx.fillStyle = T.ink3; ctx.font = '11px Inter, sans-serif';
    for (let v = 0; v <= XMAX; v += 5) ctx.fillText(v + ' m', sx(v) - 8, H - 4);
    // robot
    ctx.fillStyle = T.truth; ctx.beginPath(); ctx.arc(sx(truth), cy + 6, 8, 0, 7); ctx.fill();
    ctx.fillStyle = T.ink2; ctx.fillText('true robot', sx(truth) + 12, cy + 10);
    // sensor model
    const top = 70, mh = 50;
    ctx.beginPath(); for (let v = 0; v <= XMAX; v += 0.02) { const y = top + mh - (v > L ? 0 : pDoor(v)) * mh; v ? ctx.lineTo(sx(v), y) : ctx.moveTo(sx(v), y); }
    ctx.strokeStyle = T.s1; ctx.lineWidth = 2; ctx.stroke();
    ctx.fillStyle = T.ink2; ctx.fillText('p(door | x)', sx(L) + 8, top + 14);
    // histogram
    const bins = 100, cnt = new Array(bins).fill(0);
    for (let i = 0; i < M; i++) { const b = Math.floor(x[i] / XMAX * bins); if (b >= 0 && b < bins) cnt[b]++; }
    const hb = H - 24, ht = top + mh + 20, maxc = Math.max(M / 12, ...cnt);
    const bw = (W - 2 * pad) / bins;
    ctx.fillStyle = T.particle;
    cnt.forEach((c, b) => { const h = c / maxc * (hb - ht); if (h > 0) ctx.fillRect(pad + b * bw + 0.5, hb - h, Math.max(1, bw - 1), h); });
    ctx.strokeStyle = T.ink3; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(pad, hb); ctx.lineTo(W - pad, hb); ctx.stroke();
    ctx.fillStyle = T.ink2; ctx.fillText('particle histogram', pad, ht - 4);
    status.innerHTML = last;
  }
  new ResizeObserver(() => { resize(); draw(); }).observe(canvas.parentElement);
  UI.onTheme(draw);
  resize(); reset();
};
