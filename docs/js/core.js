/* Monte Carlo Localization core - a line-for-line port of python/mcl.
 * World frame: x right, y up, cm, origin bottom-left. grid[iy*W + ix], iy = 0 bottom row. */
'use strict';

const MCL = (() => {
  const TAU = Math.PI * 2;
  const wrap = a => { a = (a + Math.PI) % TAU; if (a < 0) a += TAU; return a - Math.PI; };
  const LASER_OFFSET = 25, MAX_RANGE = 8183, N_BEAMS = 180;
  const BEARINGS = Float64Array.from({ length: N_BEAMS }, (_, k) => (k - 90) * Math.PI / 180);

  /* ---------------------------------------------------------------- RNG */
  function makeRng(seed = 1) {
    let s = seed >>> 0;
    const rand = () => {             // mulberry32
      s = (s + 0x6D2B79F5) >>> 0;
      let t = s;
      t = Math.imul(t ^ (t >>> 15), t | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
    let spare = null;
    const normal = () => {           // Box-Muller
      if (spare !== null) { const v = spare; spare = null; return v; }
      let u = 0, v = 0;
      while (u === 0) u = rand();
      v = rand();
      const r = Math.sqrt(-2 * Math.log(u));
      spare = r * Math.sin(TAU * v);
      return r * Math.cos(TAU * v);
    };
    return { rand, normal, uniform: (a, b) => a + (b - a) * rand() };
  }

  /* ---------------------------------------------------------------- map */
  class GridMap {
    constructor(rowsBottomFirst, res) {
      this.H = rowsBottomFirst.length; this.W = rowsBottomFirst[0].length; this.res = res;
      this.width = this.W * res; this.height = this.H * res;
      this.occ = new Uint8Array(this.W * this.H);
      rowsBottomFirst.forEach((row, iy) => { for (let ix = 0; ix < this.W; ix++) this.occ[iy * this.W + ix] = row.charCodeAt(ix) === 49; });
      this.dist = edt(this.occ, this.W, this.H, res);
      const free = [];
      for (let i = 0; i < this.occ.length; i++) if (!this.occ[i]) free.push(i);
      this.freeCells = Int32Array.from(free);
    }
    inBounds(x, y) { return x >= 0 && y >= 0 && x < this.width && y < this.height; }
    isFree(x, y) {
      if (!this.inBounds(x, y)) return false;
      return !this.occ[((y / this.res) | 0) * this.W + ((x / this.res) | 0)];
    }
    distAt(x, y) {
      if (!this.inBounds(x, y)) return Infinity;
      return this.dist[((y / this.res) | 0) * this.W + ((x / this.res) | 0)];
    }
    randomFree(rng) {
      const c = this.freeCells[(rng.rand() * this.freeCells.length) | 0];
      return [((c % this.W) + rng.rand()) * this.res, (((c / this.W) | 0) + rng.rand()) * this.res];
    }
    /* sphere tracing: step by distance-to-nearest-wall (minus one cell, >= half a cell) */
    raycast(x, y, ang, maxRange = MAX_RANGE) {
      const c = Math.cos(ang), s = Math.sin(ang), half = this.res / 2;
      let t = 0;
      for (let it = 0; it < 4096; it++) {
        const px = x + t * c, py = y + t * s;
        if (!(px >= 0 && py >= 0 && px < this.width && py < this.height)) return maxRange;
        const d = this.dist[((py / this.res) | 0) * this.W + ((px / this.res) | 0)];
        if (d === 0) return Math.min(t, maxRange);
        if (t >= maxRange) return maxRange;
        t += Math.max(d - this.res, half);
      }
      return maxRange;
    }
  }

  /* exact Euclidean distance transform (Felzenszwalb & Huttenlocher), in cm */
  function edt(occ, W, H, res) {
    const INF = 1e20, f = new Float64Array(W * H);
    for (let i = 0; i < f.length; i++) f[i] = occ[i] ? 0 : INF;
    const n = Math.max(W, H), g = new Float64Array(n), d = new Float64Array(n),
      v = new Int32Array(n), z = new Float64Array(n + 1);
    const dt1 = len => {
      let k = 0; v[0] = 0; z[0] = -INF; z[1] = INF;
      for (let q = 1; q < len; q++) {
        let s = ((g[q] + q * q) - (g[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
        while (s <= z[k]) { k--; s = ((g[q] + q * q) - (g[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]); }
        k++; v[k] = q; z[k] = s; z[k + 1] = INF;
      }
      k = 0;
      for (let q = 0; q < len; q++) { while (z[k + 1] < q) k++; d[q] = (q - v[k]) ** 2 + g[v[k]]; }
    };
    for (let x = 0; x < W; x++) { for (let y = 0; y < H; y++) g[y] = f[y * W + x]; dt1(H); for (let y = 0; y < H; y++) f[y * W + x] = d[y]; }
    for (let y = 0; y < H; y++) { for (let x = 0; x < W; x++) g[x] = f[y * W + x]; dt1(W); for (let x = 0; x < W; x++) f[y * W + x] = d[x]; }
    const out = new Float32Array(W * H);
    for (let i = 0; i < f.length; i++) out[i] = Math.sqrt(f[i]) * res;
    return out;
  }

  /* ------------------------------------------------------------ motion */
  function decompose(o1, o2) {
    const dx = o2[0] - o1[0], dy = o2[1] - o1[1];
    let trans = Math.hypot(dx, dy);
    const dth = wrap(o2[2] - o1[2]);
    if (trans < 1e-3) return [0, 0, dth];
    let rot1 = wrap(Math.atan2(dy, dx) - o1[2]);
    if (Math.abs(rot1) > Math.PI / 2) { rot1 = wrap(rot1 + Math.PI); trans = -trans; }
    return [rot1, trans, wrap(dth - rot1)];
  }

  /* alphas scale VARIANCES; std = sqrt(...) */
  function sampleMotion(P, n, o1, o2, a, rng) {
    const [r1, t, r2] = decompose(o1, o2);
    if (t === 0 && r2 === 0) return;
    const s1 = Math.sqrt(a[0] * r1 * r1 + a[1] * t * t), st = Math.sqrt(a[2] * t * t + a[3] * (r1 * r1 + r2 * r2)),
      s2 = Math.sqrt(a[0] * r2 * r2 + a[1] * t * t);
    for (let i = 0; i < n; i++) {
      const R1 = r1 - s1 * rng.normal(), T = t - st * rng.normal(), R2 = r2 - s2 * rng.normal();
      const h = P[3 * i + 2] + R1;
      P[3 * i] += T * Math.cos(h); P[3 * i + 1] += T * Math.sin(h); P[3 * i + 2] = wrap(h + R2);
    }
  }

  /* ----------------------------------------------------------- sensors */
  const erf = x => {                 // Abramowitz-Stegun 7.1.26
    const s = Math.sign(x); x = Math.abs(x);
    const t = 1 / (1 + 0.3275911 * x);
    const y = 1 - (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - 0.284496736) * t + 0.254829592) * t * Math.exp(-x * x);
    return s * y;
  };
  const Phi = x => 0.5 * (1 + erf(x / Math.SQRT2));

  function beamComponents(z, zs, p) {
    const zm = p.maxRange, s = p.sigma, lam = p.lambda;
    const inside = z >= 0 && z <= zm;
    const etaH = 1 / Math.max(Phi((zm - zs) / s) - Phi(-zs / s), 1e-12);
    const hit = inside ? etaH * Math.exp(-0.5 * ((z - zs) / s) ** 2) / (Math.sqrt(TAU) * s) : 0;
    const short = (z >= 0 && z <= zs) ? lam * Math.exp(-lam * z) / Math.max(1 - Math.exp(-lam * zs), 1e-12) : 0;
    const max = z >= zm - 1 ? 1 : 0;
    const rand = (z >= 0 && z < zm) ? 1 / zm : 0;
    return { hit, short, max, rand };
  }
  function beamDensity(z, zs, p) {
    const c = beamComponents(z, zs, p);
    return p.zHit * c.hit + p.zShort * c.short + p.zMax * c.max + p.zRand * c.rand;
  }

  const SENSOR_DEFAULTS = {
    likelihood: { sigma: 10, zHit: 0.9, zRand: 0.1, stride: 5, temperature: 1, maxRange: MAX_RANGE },
    beam: { sigma: 10, zHit: 0.8, zShort: 0.05, zMax: 0.05, zRand: 0.1, lambda: 0.01, stride: 5, temperature: 1, maxRange: MAX_RANGE },
  };

  /* log p(z | x, m) for one pose; returns [logLik, beamsUsed] */
  function logLikelihood(model, p, map, x, y, th, ranges) {
    const lx = x + LASER_OFFSET * Math.cos(th), ly = y + LASER_OFFSET * Math.sin(th);
    let ll = 0, used = 0;
    if (model === 'likelihood') {
      const norm = 1 / (Math.sqrt(TAU) * p.sigma), floor = p.zRand / p.maxRange, inv = 1 / (p.sigma * p.sigma);
      for (let k = 0; k < ranges.length; k += p.stride) {
        const z = ranges[k];
        if (z >= p.maxRange - 1) continue;
        const a = th + BEARINGS[k];
        const d = map.distAt(lx + z * Math.cos(a), ly + z * Math.sin(a));
        ll += Math.log(p.zHit * norm * Math.exp(-0.5 * d * d * inv) + floor);
        used++;
      }
    } else {
      for (let k = 0; k < ranges.length; k += p.stride) {
        const zs = map.raycast(lx, ly, th + BEARINGS[k], p.maxRange);
        ll += Math.log(Math.max(beamDensity(ranges[k], zs, p), 1e-300));
        used++;
      }
    }
    return [ll / p.temperature, used];
  }

  /* synthetic scan from a true pose */
  function sampleScan(map, x, y, th, rng, o = {}) {
    const sigma = o.sigma ?? 10, pShort = o.pShort ?? 0.05, pMax = o.pMax ?? 0.03, pRand = o.pRand ?? 0.02;
    const lx = x + LASER_OFFSET * Math.cos(th), ly = y + LASER_OFFSET * Math.sin(th);
    const z = new Float64Array(N_BEAMS);
    for (let k = 0; k < N_BEAMS; k++) {
      const zs = map.raycast(lx, ly, th + BEARINGS[k]);
      if (zs >= MAX_RANGE) { z[k] = MAX_RANGE; continue; }
      const u = rng.rand();
      let v = zs + sigma * rng.normal();
      if (u < pShort) v = Math.min(-Math.log(1 - rng.rand()) / 0.01, zs);
      else if (u < pShort + pMax) v = MAX_RANGE;
      else if (u < pShort + pMax + pRand) v = rng.rand() * MAX_RANGE;
      z[k] = Math.min(Math.max(v, 0), MAX_RANGE);
    }
    return z;
  }

  /* -------------------------------------------------------- resampling */
  const RESAMPLERS = {
    low_variance(w, n, rng) {
      const idx = new Int32Array(n), r = rng.rand() / n;
      let c = w[0], i = 0;
      for (let m = 0; m < n; m++) { const u = r + m / n; while (u > c && i < n - 1) c += w[++i]; idx[m] = i; }
      return idx;
    },
    stratified(w, n, rng) {
      const idx = new Int32Array(n);
      let c = w[0], i = 0;
      for (let m = 0; m < n; m++) { const u = (m + rng.rand()) / n; while (u > c && i < n - 1) c += w[++i]; idx[m] = i; }
      return idx;
    },
    multinomial(w, n, rng) {
      const cum = new Float64Array(n); let c = 0;
      for (let i = 0; i < n; i++) { c += w[i]; cum[i] = c; }
      const idx = new Int32Array(n);
      for (let m = 0; m < n; m++) {
        const u = rng.rand() * c; let lo = 0, hi = n - 1;
        while (lo < hi) { const mid = (lo + hi) >> 1; if (cum[mid] < u) lo = mid + 1; else hi = mid; }
        idx[m] = lo;
      }
      return idx;
    },
  };

  /* ---------------------------------------------------- particle filter */
  class ParticleFilter {
    constructor(map, opts = {}) {
      this.map = map;
      this.o = Object.assign({
        n: 1000, alpha: [0.02, 1e-5, 0.01, 1.0], model: 'likelihood', sensor: null,
        resampler: 'low_variance', threshold: 0.5, minTrans: 5, minRot: 5 * Math.PI / 180,
        augmented: false, alphaSlow: 0.005, alphaFast: 0.1, seed: 1,
      }, opts);
      this.sensor = Object.assign({}, SENSOR_DEFAULTS[this.o.model], this.o.sensor || {});
      this.rng = makeRng(this.o.seed);
      this.alloc(this.o.n);
    }
    alloc(n) {
      this.n = n; this.P = new Float64Array(3 * n); this.w = new Float64Array(n).fill(1 / n);
      this.logw = new Float64Array(n); this.moved = [0, 0]; this.first = true;
      this.wSlow = 0; this.wFast = 0; this.neff = n; this.resampled = false; this.injected = 0; this.updates = 0;
    }
    initBox(xr, yr, tr) {
      const { P, n, rng, map } = this;
      for (let i = 0; i < n; i++) {
        let x, y, tries = 0;
        do { x = rng.uniform(xr[0], xr[1]); y = rng.uniform(yr[0], yr[1]); } while (!map.isFree(x, y) && ++tries < 1000);
        P[3 * i] = x; P[3 * i + 1] = y; P[3 * i + 2] = wrap(rng.uniform(tr[0], tr[1]));
      }
      this.w.fill(1 / n); this.first = true; this.moved = [0, 0]; this.wSlow = this.wFast = 0;
    }
    initGlobal() {
      const { P, n, rng, map } = this;
      for (let i = 0; i < n; i++) { const [x, y] = map.randomFree(rng); P[3 * i] = x; P[3 * i + 1] = y; P[3 * i + 2] = rng.uniform(-Math.PI, Math.PI); }
      this.w.fill(1 / n); this.first = true; this.moved = [0, 0]; this.wSlow = this.wFast = 0;
    }
    predict(o1, o2) {
      sampleMotion(this.P, this.n, o1, o2, this.o.alpha, this.rng);
      this.moved[0] += Math.hypot(o2[0] - o1[0], o2[1] - o1[1]);
      this.moved[1] += Math.abs(wrap(o2[2] - o1[2]));
    }
    update(ranges, force = false) {
      this.resampled = false; this.injected = 0;
      const o = this.o;
      if (!(force || this.first || this.moved[0] >= o.minTrans || this.moved[1] >= o.minRot)) return false;
      this.first = false; this.moved = [0, 0]; this.updates++;
      const { P, n, w, logw, map, sensor } = this;
      let mx = -Infinity, used = 0;
      for (let i = 0; i < n; i++) {
        const x = P[3 * i], y = P[3 * i + 1];
        let l;
        if (!map.isFree(x, y)) l = -Infinity;
        else { const r = logLikelihood(o.model, sensor, map, x, y, P[3 * i + 2], ranges); l = r[0]; used = r[1]; }
        l += Math.log(Math.max(w[i], 1e-300));
        logw[i] = l; if (l > mx) mx = l;
      }
      if (mx === -Infinity) { for (let i = 0; i < n; i++) logw[i] = Math.log(Math.max(w[i], 1e-300)); mx = Math.max(...logw); }
      let s = 0;
      for (let i = 0; i < n; i++) { w[i] = Math.exp(logw[i] - mx); s += w[i]; }
      let s2 = 0;
      for (let i = 0; i < n; i++) { w[i] /= s; s2 += w[i] * w[i]; }
      this.neff = 1 / s2;
      if (o.augmented && used > 0) {
        const norm = mx + Math.log(s);                   // log sum_i w_i p(z|x_i)
        const wAvg = Math.exp(norm * sensor.temperature / used);
        if (this.wSlow === 0) this.wSlow = this.wFast = wAvg;
        this.wSlow += o.alphaSlow * (wAvg - this.wSlow);
        this.wFast += o.alphaFast * (wAvg - this.wFast);
      }
      if (this.neff < o.threshold * n) this.resample();
      return true;
    }
    resample() {
      const { n, P, w, rng, map, o } = this;
      const idx = RESAMPLERS[o.resampler](w, n, rng);
      const Q = new Float64Array(3 * n);
      const pInject = (o.augmented && this.wSlow > 0) ? Math.max(0, 1 - this.wFast / this.wSlow) : 0;
      for (let m = 0; m < n; m++) {
        if (pInject > 0 && rng.rand() < pInject) {
          const [x, y] = map.randomFree(rng); Q[3 * m] = x; Q[3 * m + 1] = y; Q[3 * m + 2] = rng.uniform(-Math.PI, Math.PI);
          this.injected++;
        } else { const j = idx[m]; Q[3 * m] = P[3 * j]; Q[3 * m + 1] = P[3 * j + 1]; Q[3 * m + 2] = P[3 * j + 2]; }
      }
      this.P = Q; this.w.fill(1 / n); this.resampled = true;
      this.lastIdx = idx;
    }
    mean() {
      const { P, w, n } = this; let x = 0, y = 0, s = 0, c = 0;
      for (let i = 0; i < n; i++) { x += w[i] * P[3 * i]; y += w[i] * P[3 * i + 1]; s += w[i] * Math.sin(P[3 * i + 2]); c += w[i] * Math.cos(P[3 * i + 2]); }
      return [x, y, Math.atan2(s, c)];
    }
    /* weighted mean of the dominant mode (50 cm grid, 1 m radius) */
    estimate() {
      const { P, w, n } = this, cell = 50, mass = new Map();
      for (let i = 0; i < n; i++) { const k = Math.floor(P[3 * i] / cell) * 100003 + Math.floor(P[3 * i + 1] / cell); mass.set(k, (mass.get(k) || 0) + w[i]); }
      let best = null, bm = -1; for (const [k, m] of mass) if (m > bm) { bm = m; best = k; }
      let cx = 0, cy = 0, cw = 0;
      for (let i = 0; i < n; i++) { const k = Math.floor(P[3 * i] / cell) * 100003 + Math.floor(P[3 * i + 1] / cell); if (k === best) { cx += w[i] * P[3 * i]; cy += w[i] * P[3 * i + 1]; cw += w[i]; } }
      cx /= cw || 1; cy /= cw || 1;
      let x = 0, y = 0, s = 0, c = 0, ww = 0;
      for (let i = 0; i < n; i++) {
        if (Math.hypot(P[3 * i] - cx, P[3 * i + 1] - cy) >= 100) continue;
        const wi = w[i] + 1e-300; x += wi * P[3 * i]; y += wi * P[3 * i + 1]; s += wi * Math.sin(P[3 * i + 2]); c += wi * Math.cos(P[3 * i + 2]); ww += wi;
      }
      return [x / ww, y / ww, Math.atan2(s, c)];
    }
    spread() {
      const [mx, my] = this.mean(); const { P, w, n } = this; let v = 0;
      for (let i = 0; i < n; i++) v += w[i] * ((P[3 * i] - mx) ** 2 + (P[3 * i + 1] - my) ** 2);
      return Math.sqrt(v);
    }
  }

  return {
    wrap, makeRng, GridMap, decompose, sampleMotion, beamComponents, beamDensity, logLikelihood,
    sampleScan, RESAMPLERS, ParticleFilter, SENSOR_DEFAULTS, BEARINGS, LASER_OFFSET, MAX_RANGE, N_BEAMS,
  };
})();
