/* Tiny SVG charts with hover tooltips (no dependencies). */
'use strict';

const Charts = (() => {
  const NS = 'http://www.w3.org/2000/svg';
  const el = (tag, attrs = {}, parent) => {
    const e = document.createElementNS(NS, tag);
    for (const k in attrs) e.setAttribute(k, attrs[k]);
    if (parent) parent.appendChild(e);
    return e;
  };
  const niceTicks = (lo, hi, n = 5) => {
    const span = hi - lo || 1, step0 = span / n, mag = 10 ** Math.floor(Math.log10(step0));
    const step = [1, 2, 2.5, 5, 10].map(m => m * mag).find(s => span / s <= n) || mag * 10;
    const out = []; for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) out.push(+v.toFixed(10));
    return out;
  };
  const fmt = v => Math.abs(v) >= 1000 ? (v / 1000).toFixed(v % 1000 ? 1 : 0) + 'k' : (Math.abs(v) < 10 && v % 1 ? v.toFixed(1) : String(Math.round(v * 10) / 10));

  function frame(container, opts) {
    container.innerHTML = '';
    container.classList.add('chart');
    const W = container.clientWidth || 600, H = opts.height || 260;
    const m = Object.assign({ l: 52, r: 16, t: 14, b: 40 }, opts.margin || {});
    const svg = el('svg', { viewBox: `0 0 ${W} ${H}`, width: '100%', height: H, role: 'img', 'aria-label': opts.title || '' }, container);
    const tip = document.createElement('div'); tip.className = 'chart-tip'; container.appendChild(tip);
    return { svg, tip, W, H, m, iw: W - m.l - m.r, ih: H - m.t - m.b };
  }

  function legend(container, series) {
    if (series.length < 2) return;
    const lg = document.createElement('div'); lg.className = 'chart-legend';
    series.forEach(s => {
      const i = document.createElement('span');
      i.innerHTML = `<i style="background:${s.color};${s.dash ? 'height:0;border-top:2px dashed ' + s.color + ';background:none' : ''}"></i>${s.name}`;
      lg.appendChild(i);
    });
    container.prepend(lg);
  }

  /* series: [{name, color, data:[[x,y],...], dash}] */
  function line(container, series, opts = {}) {
    const f = frame(container, opts), { svg, tip, m, iw, ih } = f;
    const xs = series.flatMap(s => s.data.map(d => d[0])), ys = series.flatMap(s => s.data.map(d => d[1])).filter(v => v != null && isFinite(v));
    let x0 = opts.xMin ?? Math.min(...xs), x1 = opts.xMax ?? Math.max(...xs);
    let y0 = opts.yMin ?? Math.min(0, ...ys), y1 = opts.yMax ?? Math.max(...ys) * 1.08;
    const lx = opts.xLog;
    const X = v => m.l + (lx ? (Math.log(v) - Math.log(x0)) / (Math.log(x1) - Math.log(x0)) : (v - x0) / (x1 - x0)) * iw;
    const Y = v => m.t + ih - (Math.min(v, y1) - y0) / (y1 - y0) * ih;
    const g = el('g', { class: 'axis' }, svg);
    niceTicks(y0, y1).forEach(t => {
      el('line', { x1: m.l, x2: m.l + iw, y1: Y(t), y2: Y(t), class: 'grid' }, g);
      el('text', { x: m.l - 8, y: Y(t) + 4, 'text-anchor': 'end' }, g).textContent = fmt(t);
    });
    const xt = opts.xTicks || (lx ? xs.filter((v, i, a) => a.indexOf(v) === i) : niceTicks(x0, x1, 6));
    xt.forEach(t => { el('text', { x: X(t), y: m.t + ih + 18, 'text-anchor': 'middle' }, g).textContent = fmt(t); });
    el('line', { x1: m.l, x2: m.l + iw, y1: m.t + ih, y2: m.t + ih, class: 'base' }, g);
    if (opts.xLabel) el('text', { x: m.l + iw / 2, y: f.H - 4, 'text-anchor': 'middle', class: 'lbl' }, g).textContent = opts.xLabel;
    if (opts.yLabel) el('text', { x: 12, y: m.t + ih / 2, transform: `rotate(-90 12 ${m.t + ih / 2})`, 'text-anchor': 'middle', class: 'lbl' }, g).textContent = opts.yLabel;
    (opts.markers || []).forEach(mk => {
      el('line', { x1: X(mk.x), x2: X(mk.x), y1: m.t, y2: m.t + ih, class: 'marker' }, g);
      el('text', { x: X(mk.x) + 4, y: m.t + 12, class: 'lbl' }, g).textContent = mk.label;
    });
    series.forEach(s => {
      let d = '', pen = false;
      s.data.forEach(([x, y]) => {
        if (y == null || !isFinite(y)) { pen = false; return; }
        d += (pen ? 'L' : 'M') + X(x).toFixed(1) + ',' + Y(y).toFixed(1); pen = true;
      });
      el('path', { d, fill: 'none', stroke: s.color, 'stroke-width': 2, 'stroke-linejoin': 'round', 'stroke-dasharray': s.dash ? '5 4' : 'none' }, svg);
      if (s.points) s.data.forEach(([x, y]) => y != null && el('circle', { cx: X(x), cy: Y(y), r: 4, fill: s.color, stroke: 'var(--surface)', 'stroke-width': 2 }, svg));
    });
    // hover crosshair
    const cross = el('line', { y1: m.t, y2: m.t + ih, class: 'cross', visibility: 'hidden' }, svg);
    const dots = series.map(s => el('circle', { r: 4, fill: s.color, stroke: 'var(--surface)', 'stroke-width': 2, visibility: 'hidden' }, svg));
    const hit = el('rect', { x: m.l, y: m.t, width: iw, height: ih, fill: 'transparent' }, svg);
    const move = ev => {
      const r = svg.getBoundingClientRect(), px = (ev.clientX - r.left) * f.W / r.width;
      let best = null, bd = Infinity;
      series[0].data.forEach(([x], i) => { const dd = Math.abs(X(x) - px); if (dd < bd) { bd = dd; best = i; } });
      if (best == null) return;
      const xv = series[0].data[best][0];
      cross.setAttribute('x1', X(xv)); cross.setAttribute('x2', X(xv)); cross.setAttribute('visibility', 'visible');
      let html = `<b>${opts.xName || opts.xLabel || 'x'}: ${fmt(xv)}</b>`;
      series.forEach((s, k) => {
        const pt = s.data.find(d => d[0] === xv) || s.data[best];
        if (pt && pt[1] != null && isFinite(pt[1])) {
          dots[k].setAttribute('cx', X(pt[0])); dots[k].setAttribute('cy', Y(pt[1])); dots[k].setAttribute('visibility', 'visible');
          html += `<br><i style="background:${s.color}"></i>${s.name}: ${(opts.yFmt || fmt)(pt[1])}${opts.yUnit || ''}`;
        } else dots[k].setAttribute('visibility', 'hidden');
      });
      tip.innerHTML = html; tip.style.opacity = 1;
      const tx = X(xv) * r.width / f.W;
      tip.style.left = Math.min(tx + 12, r.width - tip.offsetWidth - 4) + 'px'; tip.style.top = '8px';
    };
    hit.addEventListener('pointermove', move);
    hit.addEventListener('pointerleave', () => { tip.style.opacity = 0; cross.setAttribute('visibility', 'hidden'); dots.forEach(d => d.setAttribute('visibility', 'hidden')); });
    legend(container, series);
    return { X, Y, svg };
  }

  /* grouped bars: groups:[label], series:[{name,color,values:[..]}] */
  function bars(container, groups, series, opts = {}) {
    const f = frame(container, opts), { svg, tip, m, iw, ih } = f;
    const y1 = opts.yMax ?? Math.max(...series.flatMap(s => s.values)) * 1.1, y0 = 0;
    const Y = v => m.t + ih - (v - y0) / (y1 - y0) * ih;
    const g = el('g', { class: 'axis' }, svg);
    niceTicks(y0, y1).forEach(t => {
      el('line', { x1: m.l, x2: m.l + iw, y1: Y(t), y2: Y(t), class: 'grid' }, g);
      el('text', { x: m.l - 8, y: Y(t) + 4, 'text-anchor': 'end' }, g).textContent = (opts.yTickFmt || fmt)(t);
    });
    const gw = iw / groups.length, bw = Math.min(34, (gw * 0.72) / series.length);
    groups.forEach((gl, gi) => {
      el('text', { x: m.l + gw * (gi + 0.5), y: m.t + ih + 18, 'text-anchor': 'middle' }, g).textContent = gl;
      series.forEach((s, si) => {
        const v = s.values[gi], x = m.l + gw * (gi + 0.5) - (series.length * (bw + 2)) / 2 + si * (bw + 2);
        const h = Math.max(0, Y(y0) - Y(v));
        const r = Math.min(4, h / 2, bw / 2);
        const p = el('path', { d: `M${x},${Y(y0)}V${Y(v) + r}q0,-${r} ${r},-${r}h${bw - 2 * r}q${r},0 ${r},${r}V${Y(y0)}Z`, fill: s.color, class: 'bar' }, svg);
        if (opts.labels) el('text', { x: x + bw / 2, y: Y(v) - 5, 'text-anchor': 'middle', class: 'val' }, svg).textContent = (opts.valFmt || fmt)(v);
        const show = ev => {
          const rr = svg.getBoundingClientRect();
          tip.innerHTML = `<b>${gl}</b><br><i style="background:${s.color}"></i>${s.name}: ${(opts.valFmt || fmt)(v)}${opts.yUnit || ''}`;
          tip.style.opacity = 1; tip.style.left = Math.min((x + bw) * rr.width / f.W + 8, rr.width - 170) + 'px'; tip.style.top = '8px';
        };
        const hitR = el('rect', { x: x - 2, y: m.t, width: bw + 4, height: ih, fill: 'transparent' }, svg);
        hitR.addEventListener('pointerenter', show); hitR.addEventListener('pointerleave', () => { tip.style.opacity = 0; });
        void p;
      });
    });
    el('line', { x1: m.l, x2: m.l + iw, y1: Y(0), y2: Y(0), class: 'base' }, g);
    if (opts.xLabel) el('text', { x: m.l + iw / 2, y: f.H - 4, 'text-anchor': 'middle', class: 'lbl' }, g).textContent = opts.xLabel;
    if (opts.yLabel) el('text', { x: 12, y: m.t + ih / 2, transform: `rotate(-90 12 ${m.t + ih / 2})`, 'text-anchor': 'middle', class: 'lbl' }, g).textContent = opts.yLabel;
    legend(container, series);
  }

  return { line, bars };
})();
