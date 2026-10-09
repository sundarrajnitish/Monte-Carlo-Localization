/* Motion model, beam model, likelihood field, resampling widgets. */
'use strict';

/* =============================================================== MOTION */
Widgets.motion = (root) => {
  const canvas = root.querySelector('canvas'), read = document.getElementById('motion-read');
  const vp = new UI.Viewport(canvas, null, { aspect: 2.2, minH: 280, maxH: 440, pan: false });
  const start = [60, 100, 0], goal = [300, 170, 0.6];
  let alpha = [0.02, 1e-5, 0.01, 1.0], N = 600, seed = 1;
  const fitView = () => { vp.cx = 200; vp.cy = 130; vp.scale = Math.min(vp.w / 420, vp.h / 280); };
  fitView();
  vp.addHandle({ get: () => [goal[0], goal[1]], set: (x, y) => { goal[0] = x; goal[1] = y; draw(); }, r: 22 });
  vp.addHandle({ get: () => [goal[0] + 30 * Math.cos(goal[2]), goal[1] + 30 * Math.sin(goal[2])], set: (x, y) => { goal[2] = Math.atan2(y - goal[1], x - goal[0]); draw(); }, r: 16 });
  const ctl = document.getElementById('motion-ctl'), btn = document.getElementById('motion-btn');
  const sl = [
    UI.slider(ctl, { label: 'α₁ rot ← rot', min: 1e-4, max: 1, value: alpha[0], log: true, fmt: v => v.toExponential(1), onInput: v => { alpha[0] = v; draw(); } }),
    UI.slider(ctl, { label: 'α₂ rot ← trans', min: 1e-7, max: 1e-3, value: alpha[1], log: true, fmt: v => v.toExponential(1), onInput: v => { alpha[1] = v; draw(); } }),
    UI.slider(ctl, { label: 'α₃ trans ← trans', min: 1e-4, max: 0.5, value: alpha[2], log: true, fmt: v => v.toExponential(1), onInput: v => { alpha[2] = v; draw(); } }),
    UI.slider(ctl, { label: 'α₄ trans ← rot', min: 1e-2, max: 1e4, value: alpha[3], log: true, fmt: v => v.toExponential(1), onInput: v => { alpha[3] = v; draw(); } }),
  ];
  UI.slider(ctl, { label: 'Samples', min: 50, max: 3000, step: 50, value: N, onInput: v => { N = v; draw(); } });
  const preset = a => () => { alpha = a.slice(); sl.forEach((s, i) => s.set(alpha[i])); draw(); };
  UI.button(btn, 'Tuned (used here)', preset([0.02, 1e-5, 0.01, 1.0]));
  UI.button(btn, 'Slippery wheels', preset([0.02, 1e-5, 0.15, 1.0]));
  UI.button(btn, 'Bad gyro → banana', preset([0.02, 2e-4, 0.002, 1.0]));
  UI.button(btn, 'Almost noise-free', preset([1e-6, 1e-6, 1e-6, 1e-6]));
  UI.button(btn, '↻ Resample noise', () => { seed++; draw(); });

  function draw() {
    const T = UI.T(), ctx = vp.begin();
    // grid
    ctx.strokeStyle = T.grid; ctx.lineWidth = 1;
    const [wx0, wy1] = vp.toWorld(0, 0), [wx1, wy0] = vp.toWorld(vp.w, vp.h);
    for (let gx = Math.floor(wx0 / 20) * 20; gx <= wx1; gx += 20) { const [a, b] = vp.toScreen(gx, wy0), [c, d] = vp.toScreen(gx, wy1); ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke(); }
    for (let gy = Math.floor(wy0 / 20) * 20; gy <= wy1; gy += 20) { const [a, b] = vp.toScreen(wx0, gy), [c, d] = vp.toScreen(wx1, gy); ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke(); }
    const P = new Float64Array(3 * N);
    for (let i = 0; i < N; i++) { P[3 * i] = start[0]; P[3 * i + 1] = start[1]; P[3 * i + 2] = start[2]; }
    // The filter applies the model once per scan (~5 cm of travel), so chain
    // 5 cm steps: turn, drive in small increments, turn.
    const rng = MCL.makeRng(seed), [q1, qt] = MCL.decompose(start, goal), hd = start[2] + q1;
    const chain = [start, [start[0], start[1], hd]];
    const nStep = Math.max(1, Math.ceil(Math.abs(qt) / 5));
    for (let j = 1; j <= nStep; j++) chain.push([start[0] + qt * j / nStep * Math.cos(hd), start[1] + qt * j / nStep * Math.sin(hd), hd]);
    chain.push(goal.slice());
    for (let j = 1; j < chain.length; j++) MCL.sampleMotion(P, N, chain[j - 1], chain[j], alpha, rng);
    vp.particles(P, N, T.particle, null, 1.3);
    const [r1, t, r2] = MCL.decompose(start, goal);
    // decomposition drawing
    const h1 = start[2] + r1, mid = [start[0] + t * Math.cos(h1), start[1] + t * Math.sin(h1)];
    vp.path([start, mid], T.s1, 2, [6, 4], Infinity);
    const arc = (c, a0, a1, rad, label) => {
      const [sx, sy] = vp.toScreen(c[0], c[1]), R = rad * vp.scale;
      ctx.beginPath(); ctx.arc(sx, sy, R, -a0, -a1, a1 > a0); ctx.strokeStyle = T.s1; ctx.lineWidth = 2; ctx.stroke();
      ctx.fillStyle = T.ink; ctx.font = '600 12px Inter, sans-serif';
      const am = (a0 + a1) / 2; ctx.fillText(label, sx + (R + 8) * Math.cos(am), sy - (R + 8) * Math.sin(am));
    };
    if (Math.abs(r1) > 0.01) arc(start, start[2], h1, 34, 'δrot1');
    if (Math.abs(r2) > 0.01) arc(goal, h1, goal[2], 28, 'δrot2');
    const [mx, my] = vp.toScreen((start[0] + mid[0]) / 2, (start[1] + mid[1]) / 2);
    ctx.fillText('δtrans', mx + 6, my - 8);
    vp.pose(start[0], start[1], start[2], T.ink3, 10);
    vp.pose(goal[0], goal[1], goal[2], T.truth, 10);
    const [hx, hy] = vp.toScreen(goal[0] + 30 * Math.cos(goal[2]), goal[1] + 30 * Math.sin(goal[2]));
    ctx.fillStyle = T.ink; ctx.beginPath(); ctx.arc(hx, hy, 6, 0, 7); ctx.fill();
    // sample spread
    let sx = 0, sy = 0; for (let i = 0; i < N; i++) { sx += P[3 * i]; sy += P[3 * i + 1]; } sx /= N; sy /= N;
    let v = 0; for (let i = 0; i < N; i++) v += (P[3 * i] - sx) ** 2 + (P[3 * i + 1] - sy) ** 2;
    read.innerHTML = `δrot1 <b>${UI.fmtDeg(r1)}</b> · δtrans <b>${t.toFixed(0)} cm</b> · δrot2 <b>${UI.fmtDeg(r2)}</b><br>` +
      `applied as ${nStep} steps of ≤5 cm · position spread <b>${Math.sqrt(v / N).toFixed(1)} cm</b>`;
  }
  vp.onChange = () => { fitView(); draw(); }; UI.onTheme(draw);
  draw();
};

/* ================================================================= BEAM */
Widgets.beam = (root) => {
  const canvas = root.querySelector('canvas'), ctx = canvas.getContext('2d');
  const p = { maxRange: 1000, sigma: 20, lambda: 0.005, zHit: 0.7, zShort: 0.15, zMax: 0.05, zRand: 0.1 };
  let zs = 600;
  const ctl = document.getElementById('beam-ctl');
  UI.slider(ctl, { label: 'expected range z*', min: 50, max: 990, step: 10, value: zs, fmt: v => v + ' cm', onInput: v => { zs = v; draw(); } });
  UI.slider(ctl, { label: 'σ<sub>hit</sub>', min: 2, max: 100, step: 1, value: p.sigma, fmt: v => v + ' cm', onInput: v => { p.sigma = v; draw(); } });
  UI.slider(ctl, { label: 'λ<sub>short</sub>', min: 0.0005, max: 0.05, value: p.lambda, log: true, fmt: v => v.toFixed(4), onInput: v => { p.lambda = v; draw(); } });
  ['zHit', 'zShort', 'zMax', 'zRand'].forEach(k => UI.slider(ctl, { label: `z<sub>${k.slice(1).toLowerCase()}</sub> (weight)`, min: 0, max: 1, step: 0.01, value: p[k], fmt: v => v.toFixed(2), onInput: v => { p[k] = v; draw(); } }));
  function resize() {
    const dpr = Math.min(devicePixelRatio || 1, 2), w = canvas.parentElement.clientWidth, h = Math.max(220, Math.min(300, w * 0.34));
    canvas.width = w * dpr; canvas.height = h * dpr; canvas.style.width = w + 'px'; canvas.style.height = h + 'px'; ctx.setTransform(dpr, 0, 0, dpr, 0, 0); canvas._w = w; canvas._h = h;
  }
  function draw() {
    const T = UI.T(), W = canvas._w, H = canvas._h, l = 44, r = 18, t = 14, b = 30;
    const s = p.zHit + p.zShort + p.zMax + p.zRand || 1, q = Object.assign({}, p, { zHit: p.zHit / s, zShort: p.zShort / s, zMax: p.zMax / s, zRand: p.zRand / s });
    ctx.fillStyle = T.surface2; ctx.fillRect(0, 0, W, H);
    const zsN = [], comp = { hit: [], short: [], rand: [] }, mix = [];
    let ymax = 0;
    for (let z = 0; z <= p.maxRange - 2; z += 2) {
      const c = MCL.beamComponents(z, zs, p);
      const m = q.zHit * c.hit + q.zShort * c.short + q.zRand * c.rand;
      zsN.push(z); mix.push(m); comp.hit.push(q.zHit * c.hit); comp.short.push(q.zShort * c.short); comp.rand.push(q.zRand * c.rand);
      ymax = Math.max(ymax, m);
    }
    ymax *= 1.15;
    const X = z => l + z / p.maxRange * (W - l - r), Y = v => H - b - v / ymax * (H - t - b);
    ctx.strokeStyle = T.grid; ctx.fillStyle = T.ink3; ctx.font = '11px Inter, sans-serif'; ctx.lineWidth = 1;
    for (let z = 0; z <= p.maxRange; z += 200) { ctx.fillText(z, X(z) - 8, H - 12); }
    ctx.beginPath(); ctx.moveTo(l, H - b); ctx.lineTo(W - r, H - b); ctx.strokeStyle = T.ink3; ctx.stroke();
    ctx.fillText('measured range z (cm)', W / 2 - 50, H - 1);
    const line = (arr, col, dash, w = 2) => { ctx.beginPath(); arr.forEach((v, i) => i ? ctx.lineTo(X(zsN[i]), Y(v)) : ctx.moveTo(X(zsN[i]), Y(v))); ctx.strokeStyle = col; ctx.setLineDash(dash || []); ctx.lineWidth = w; ctx.stroke(); ctx.setLineDash([]); };
    // fill mixture
    ctx.beginPath(); ctx.moveTo(X(0), Y(0)); mix.forEach((v, i) => ctx.lineTo(X(zsN[i]), Y(v))); ctx.lineTo(X(zsN[zsN.length - 1]), Y(0)); ctx.closePath();
    ctx.fillStyle = T.s1; ctx.globalAlpha = 0.1; ctx.fill(); ctx.globalAlpha = 1;
    line(comp.hit, T.s1, [5, 4]); line(comp.short, T.s2, [5, 4]); line(comp.rand, T.s3, [5, 4]); line(mix, T.ink, null, 2.2);
    // max spike
    const spikeH = Math.min(H - t - b, q.zMax * 3000 / p.maxRange / ymax * (H - t - b));
    ctx.fillStyle = T.s4; ctx.fillRect(X(p.maxRange) - 5, H - b - spikeH, 6, spikeH);
    // z* marker
    ctx.strokeStyle = T.ink3; ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(X(zs), t); ctx.lineTo(X(zs), H - b); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = T.ink; ctx.font = '600 12px Inter, sans-serif'; ctx.fillText('z*', X(zs) + 4, t + 10);
  }
  new ResizeObserver(() => { resize(); draw(); }).observe(canvas.parentElement); UI.onTheme(draw);
  resize(); draw();
};

/* =================================================== LIKELIHOOD FIELD */
Widgets.field = (root, D) => {
  const canvas = root.querySelector('canvas'), read = document.getElementById('field-read');
  const vp = new UI.Viewport(canvas, D.map, { aspect: 3.2, minH: 240, maxH: 440 });
  let sigma = 10, k = 150, img = null;
  vp.fit([D.reference.lf[k][0] - 650, -10, D.reference.lf[k][0] + 650, D.map.height + 10]);
  const ctl = document.getElementById('field-ctl');
  UI.slider(ctl, { label: 'σ<sub>hit</sub> (field width)', min: 2, max: 60, step: 1, value: sigma, fmt: v => v + ' cm', onInput: v => { sigma = v; img = null; draw(); } });
  UI.slider(ctl, { label: 'Scan', min: 0, max: D.scans.length - 1, step: 1, value: k, fmt: v => '#' + v, onInput: v => { k = v; draw(); } });
  const build = () => {
    const m = D.map, off = document.createElement('canvas'); off.width = m.W; off.height = m.H;
    const g = off.getContext('2d'), im = g.createImageData(m.W, m.H), T = UI.T();
    const hex = h => { const v = parseInt(h.slice(1), 16); return [v >> 16 & 255, v >> 8 & 255, v & 255]; };
    const lo = hex(T.floor), hi = hex(T.s1), wall = hex(T.wall);
    for (let iy = 0; iy < m.H; iy++) for (let ix = 0; ix < m.W; ix++) {
      const d = m.dist[iy * m.W + ix], a = Math.exp(-0.5 * (d / sigma) ** 2), p = ((m.H - 1 - iy) * m.W + ix) * 4;
      const c = d === 0 ? wall : lo.map((v, j) => Math.round(v + (hi[j] - v) * a));
      im.data[p] = c[0]; im.data[p + 1] = c[1]; im.data[p + 2] = c[2]; im.data[p + 3] = 255;
    }
    g.putImageData(im, 0, 0); return off;
  };
  function draw() {
    if (!img) img = build();
    const T = UI.T(); vp.begin(); vp.drawMap(img);
    const r = D.reference.lf[k];
    vp.scan(r[0], r[1], r[2], D.scans[k][4], T.particle, 2, false, T.particle);
    vp.pose(r[0], r[1], r[2], T.ink3, 14);
  }
  canvas.addEventListener('pointermove', e => {
    const b = canvas.getBoundingClientRect(), [x, y] = vp.toWorld(e.clientX - b.left, e.clientY - b.top), d = D.map.distAt(x, y);
    read.innerHTML = isFinite(d) ? `(${x.toFixed(0)}, ${y.toFixed(0)}) cm<br>distance to wall <b>${d.toFixed(0)} cm</b><br>field value <b>${Math.exp(-0.5 * (d / sigma) ** 2).toFixed(3)}</b>` : 'outside the map';
  });
  vp.onChange = draw; UI.onTheme(() => { img = null; draw(); });
  draw();
};

/* ========================================================== RESAMPLING */
Widgets.resample = (root) => {
  const canvas = root.querySelector('canvas'), ctx = canvas.getContext('2d'), status = document.getElementById('rs-status');
  let M = 16, peak = 2.5, scheme = 'low_variance', rng = MCL.makeRng(11), w = [], ptr = [], counts = null, stats = null;
  const newWeights = () => {
    const raw = Array.from({ length: M }, () => Math.exp(peak * rng.normal()));
    const s = raw.reduce((a, b) => a + b, 0); w = raw.map(v => v / s); ptr = []; counts = null; stats = null; draw();
  };
  const pointers = () => {
    const n = M, u = [];
    if (scheme === 'low_variance') { const r = rng.rand() / n; for (let m = 0; m < n; m++) u.push(r + m / n); }
    else if (scheme === 'stratified') for (let m = 0; m < n; m++) u.push((m + rng.rand()) / n);
    else for (let m = 0; m < n; m++) u.push(rng.rand());
    return u;
  };
  const pick = u => { let c = 0; for (let i = 0; i < M; i++) { c += w[i]; if (u <= c) return i; } return M - 1; };
  const doResample = () => { ptr = pointers(); counts = new Array(M).fill(0); ptr.forEach(u => counts[pick(u)]++); draw(); };
  const trials = () => {
    const S = 2000, sum = new Array(M).fill(0), sq = new Array(M).fill(0); let lost = 0;
    for (let t = 0; t < S; t++) {
      const c = new Array(M).fill(0); pointers().forEach(u => c[pick(u)]++);
      c.forEach((v, i) => { sum[i] += v; sq[i] += v * v; if (v === 0 && w[i] * M >= 1) lost++; });
    }
    stats = { mean: sum.map(v => v / S), sd: sum.map((v, i) => Math.sqrt(Math.max(0, sq[i] / S - (v / S) ** 2))), lost: lost / S };
    draw();
  };
  const btn = document.getElementById('rs-btn');
  UI.button(btn, 'Resample once', doResample, 'primary');
  UI.button(btn, 'Repeat 2000× (variance)', trials);
  UI.button(btn, 'New weights', newWeights);
  const ctl = document.getElementById('rs-ctl');
  UI.select(ctl, { label: 'Scheme', value: scheme, options: [['low_variance', 'Low-variance (systematic)'], ['stratified', 'Stratified'], ['multinomial', 'Multinomial']], onChange: v => { scheme = v; ptr = []; counts = null; if (stats) trials(); else draw(); } });
  UI.slider(ctl, { label: 'Weight skew', min: 0, max: 4, step: 0.1, value: peak, fmt: v => v.toFixed(1), onInput: v => { peak = v; newWeights(); } });
  UI.slider(ctl, { label: 'Particles M', min: 4, max: 40, step: 1, value: M, onInput: v => { M = v; newWeights(); } });
  function resize() {
    const dpr = Math.min(devicePixelRatio || 1, 2), wd = canvas.parentElement.clientWidth, h = 250;
    canvas.width = wd * dpr; canvas.height = h * dpr; canvas.style.width = wd + 'px'; canvas.style.height = h + 'px'; ctx.setTransform(dpr, 0, 0, dpr, 0, 0); canvas._w = wd; canvas._h = h;
  }
  function draw() {
    const T = UI.T(), W = canvas._w, H = canvas._h, l = 16, r = 16, top = 46, sh = 34;
    ctx.fillStyle = T.surface2; ctx.fillRect(0, 0, W, H);
    const X = u => l + u * (W - l - r);
    let c = 0;
    const cols = [T.s1, T.s2, T.s3, T.s4, T.s5];
    w.forEach((wi, i) => {
      ctx.fillStyle = cols[i % 5]; ctx.globalAlpha = 0.85; ctx.fillRect(X(c) + 1, top, Math.max(1, X(c + wi) - X(c) - 2), sh); ctx.globalAlpha = 1;
      if (X(c + wi) - X(c) > 18) { ctx.fillStyle = '#fff'; ctx.font = '600 11px Inter, sans-serif'; ctx.fillText(i, X(c + wi / 2) - 4, top + 21); }
      c += wi;
    });
    ctx.fillStyle = T.ink2; ctx.font = '12px Inter, sans-serif';
    ctx.fillText('cumulative weight 0 → 1 (width of a segment = particle weight)', l, top - 28);
    ptr.forEach(u => { ctx.strokeStyle = T.ink; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(X(u), top - 16); ctx.lineTo(X(u), top + sh + 2); ctx.stroke(); ctx.beginPath(); ctx.moveTo(X(u) - 4, top - 16); ctx.lineTo(X(u) + 4, top - 16); ctx.lineTo(X(u), top - 8); ctx.fill(); });
    // counts per particle vs expected M*w
    const base = H - 26, bw = (W - l - r) / M, maxv = Math.max(4, ...w.map(v => v * M + 1), ...(counts || []));
    const Yc = v => base - v / maxv * (base - top - sh - 30);
    for (let i = 0; i < M; i++) {
      const x0 = l + i * bw;
      ctx.fillStyle = cols[i % 5];
      if (stats) {
        const m = stats.mean[i], sd = stats.sd[i];
        ctx.globalAlpha = 0.85; ctx.fillRect(x0 + bw * 0.25, Yc(m), bw * 0.5, base - Yc(m)); ctx.globalAlpha = 1;
        ctx.strokeStyle = T.ink; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(x0 + bw / 2, Yc(m - sd)); ctx.lineTo(x0 + bw / 2, Yc(m + sd)); ctx.stroke();
      } else if (counts) {
        const rr = Math.max(2, Math.min(5, (Yc(0) - Yc(1)) / 2 - 1));
        for (let k = 0; k < counts[i]; k++) { ctx.beginPath(); ctx.arc(x0 + bw / 2, Yc(k + 0.5), rr, 0, 7); ctx.fill(); }
      }
      ctx.strokeStyle = T.ink3; ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(x0 + 2, Yc(w[i] * M)); ctx.lineTo(x0 + bw - 2, Yc(w[i] * M)); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = T.ink3; ctx.font = '10px Inter, sans-serif'; if (bw > 14) ctx.fillText(i, x0 + bw / 2 - 3, H - 10);
    }
    const neff = 1 / w.reduce((a, b) => a + b * b, 0);
    let txt = `N<sub>eff</sub> = <b>${neff.toFixed(1)}</b> of ${M}. Dashed line: expected copies M·w.`;
    if (counts) txt += ` This draw kept <b>${counts.filter(v => v > 0).length}</b> distinct particles.`;
    if (stats) txt += ` Over 2000 draws: bar = mean copies, whisker = ±1 sd. Particles that deserved at least one copy (M·w ≥ 1) but got none: <b>${stats.lost.toFixed(2)}</b> per draw.`;
    status.innerHTML = txt;
  }
  new ResizeObserver(() => { resize(); draw(); }).observe(canvas.parentElement); UI.onTheme(draw);
  resize(); newWeights();
};
