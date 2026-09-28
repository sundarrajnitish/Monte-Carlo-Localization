/* Shared UI: theme tokens, map viewport, controls. */
'use strict';

const UI = (() => {
  const css = name => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  let tokens = {};
  const readTokens = () => {
    tokens = {
      surface: css('--surface'), surface2: css('--surface-2'), ink: css('--ink'), ink2: css('--ink-2'), ink3: css('--ink-3'),
      wall: css('--wall'), floor: css('--floor'), grid: css('--grid'),
      s1: css('--series-1'), s2: css('--series-2'), s3: css('--series-3'), s4: css('--series-4'), s5: css('--series-5'),
      particle: css('--particle'), truth: css('--truth'), ray: css('--ray'), odom: css('--odom'),
    };
    return tokens;
  };
  const T = () => tokens;
  const listeners = [];
  const onTheme = fn => listeners.push(fn);
  const themeChanged = () => { readTokens(); listeners.forEach(f => f()); };

  /* ------------------------------------------------------------ viewport */
  class Viewport {
    constructor(canvas, map, opts = {}) {
      this.c = canvas; this.ctx = canvas.getContext('2d'); this.map = map;
      this.opts = Object.assign({ minH: 160, maxH: 520, aspect: null, pan: true }, opts);
      this.scale = 1; this.cx = map ? map.width / 2 : 0; this.cy = map ? map.height / 2 : 0;
      this.mapImg = null; this.handles = []; this.drag = null; this.onChange = null;
      this.resize(); if (map) this.fit();
      new ResizeObserver(() => { this.resize(); this.onChange && this.onChange(); }).observe(canvas.parentElement);
      this.bind();
    }
    resize() {
      const dpr = Math.min(window.devicePixelRatio || 1, 2), w = this.c.parentElement.clientWidth;
      let h = this.opts.aspect ? w / this.opts.aspect : this.opts.height || 300;
      h = Math.max(this.opts.minH, Math.min(this.opts.maxH, h));
      this.w = w; this.h = h;
      this.c.width = Math.round(w * dpr); this.c.height = Math.round(h * dpr);
      this.c.style.width = w + 'px'; this.c.style.height = h + 'px';
      this.dpr = dpr;
    }
    fit(box) {
      const b = box || [0, 0, this.map.width, this.map.height];
      this.cx = (b[0] + b[2]) / 2; this.cy = (b[1] + b[3]) / 2;
      this.scale = Math.min(this.w / (b[2] - b[0]), this.h / (b[3] - b[1])) * 0.98;
    }
    zoom(f, px = this.w / 2, py = this.h / 2) {
      const [wx, wy] = this.toWorld(px, py);
      this.scale = Math.max(0.05, Math.min(20, this.scale * f));
      const [nx, ny] = this.toWorld(px, py);
      this.cx += wx - nx; this.cy += wy - ny;
    }
    follow(x, y, k = 0.15) { this.cx += (x - this.cx) * k; this.cy += (y - this.cy) * k; }
    toScreen(x, y) { return [(x - this.cx) * this.scale + this.w / 2, this.h / 2 - (y - this.cy) * this.scale]; }
    toWorld(px, py) { return [(px - this.w / 2) / this.scale + this.cx, (this.h / 2 - py) / this.scale + this.cy]; }
    begin() {
      const { ctx } = this;
      ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
      ctx.fillStyle = T().surface2; ctx.fillRect(0, 0, this.w, this.h);
      return ctx;
    }
    buildMapImage() {
      const m = this.map, off = document.createElement('canvas');
      off.width = m.W; off.height = m.H;
      const g = off.getContext('2d'), img = g.createImageData(m.W, m.H);
      const hex = h => { const v = parseInt(h.replace('#', ''), 16); return [v >> 16 & 255, v >> 8 & 255, v & 255]; };
      const wall = hex(T().wall), floor = hex(T().floor);
      for (let iy = 0; iy < m.H; iy++) for (let ix = 0; ix < m.W; ix++) {
        const o = m.occ[iy * m.W + ix], p = ((m.H - 1 - iy) * m.W + ix) * 4, c = o ? wall : floor;
        img.data[p] = c[0]; img.data[p + 1] = c[1]; img.data[p + 2] = c[2]; img.data[p + 3] = 255;
      }
      g.putImageData(img, 0, 0);
      this.mapImg = off;
    }
    drawMap(overlay) {
      if (!this.mapImg) this.buildMapImage();
      const { ctx } = this, [x0, y0] = this.toScreen(0, this.map.height);
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(overlay || this.mapImg, x0, y0, this.map.width * this.scale, this.map.height * this.scale);
    }
    /* ------------------------------------------------------ primitives */
    pose(x, y, th, color, r = 14, fill = true) {
      const { ctx } = this, [sx, sy] = this.toScreen(x, y), rr = Math.max(4, r * this.scale);
      ctx.save(); ctx.translate(sx, sy); ctx.rotate(-th);
      ctx.beginPath(); ctx.arc(0, 0, rr, 0, Math.PI * 2);
      ctx.fillStyle = fill ? color : 'transparent'; ctx.globalAlpha = fill ? 0.9 : 1; ctx.fill();
      ctx.globalAlpha = 1; ctx.lineWidth = 2; ctx.strokeStyle = fill ? T().surface : color; ctx.stroke();
      ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(rr * 1.9, 0); ctx.strokeStyle = fill ? color : color; ctx.lineWidth = 2.5; ctx.stroke();
      ctx.restore();
    }
    particles(P, n, color, w, size = 1.6) {
      const { ctx } = this; ctx.fillStyle = color;
      const s = this.scale, hw = this.w / 2, hh = this.h / 2, len = Math.max(3, 6 * s);
      ctx.strokeStyle = color; ctx.lineWidth = 1; ctx.globalAlpha = 0.55;
      ctx.beginPath();
      for (let i = 0; i < n; i++) {
        const sx = (P[3 * i] - this.cx) * s + hw, sy = hh - (P[3 * i + 1] - this.cy) * s;
        if (sx < -10 || sy < -10 || sx > this.w + 10 || sy > this.h + 10) continue;
        const th = P[3 * i + 2];
        ctx.moveTo(sx, sy); ctx.lineTo(sx + len * Math.cos(th), sy - len * Math.sin(th));
      }
      ctx.stroke();
      ctx.globalAlpha = 0.9;
      for (let i = 0; i < n; i++) {
        const sx = (P[3 * i] - this.cx) * s + hw, sy = hh - (P[3 * i + 1] - this.cy) * s;
        if (sx < -10 || sy < -10 || sx > this.w + 10 || sy > this.h + 10) continue;
        const r = w ? Math.max(1.2, Math.min(5, size * Math.sqrt(w[i] * n))) : size;
        ctx.fillRect(sx - r, sy - r, 2 * r, 2 * r);
      }
      ctx.globalAlpha = 1;
    }
    path(pts, color, width = 2, dash, maxGap = 150) {
      if (!pts.length) return;
      const { ctx } = this; ctx.beginPath();
      pts.forEach((p, i) => {
        const [sx, sy] = this.toScreen(p[0], p[1]);
        const jump = i && Math.hypot(p[0] - pts[i - 1][0], p[1] - pts[i - 1][1]) > maxGap;   // don't draw teleports
        i && !jump ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy);
      });
      ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash || []); ctx.lineJoin = 'round'; ctx.stroke(); ctx.setLineDash([]);
    }
    scan(x, y, th, ranges, color, stride = 2, rays = true, dotColor) {
      const { ctx } = this, lx = x + MCL.LASER_OFFSET * Math.cos(th), ly = y + MCL.LASER_OFFSET * Math.sin(th);
      const [sx, sy] = this.toScreen(lx, ly);
      if (rays) {
        ctx.beginPath(); ctx.strokeStyle = color; ctx.globalAlpha = 0.25; ctx.lineWidth = 1;
        for (let k = 0; k < ranges.length; k += stride) {
          const z = Math.min(ranges[k], 3000), a = th + MCL.BEARINGS[k];
          const [ex, ey] = this.toScreen(lx + z * Math.cos(a), ly + z * Math.sin(a));
          ctx.moveTo(sx, sy); ctx.lineTo(ex, ey);
        }
        ctx.stroke(); ctx.globalAlpha = 1;
      }
      ctx.fillStyle = dotColor || color;
      for (let k = 0; k < ranges.length; k += stride) {
        if (ranges[k] >= MCL.MAX_RANGE - 1) continue;
        const a = th + MCL.BEARINGS[k];
        const [ex, ey] = this.toScreen(lx + ranges[k] * Math.cos(a), ly + ranges[k] * Math.sin(a));
        ctx.beginPath(); ctx.arc(ex, ey, 2.4, 0, Math.PI * 2); ctx.fill();
      }
    }
    /* ------------------------------------------------------ interaction */
    addHandle(h) { this.handles.push(h); }   // {get:()=>[x,y], set:(x,y)=>{}, r: px}
    bind() {
      const c = this.c;
      c.style.touchAction = this.opts.pan ? 'none' : 'pan-y';
      const pos = e => { const r = c.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; };
      c.addEventListener('pointerdown', e => {
        const [px, py] = pos(e);
        for (const h of this.handles) {
          const [hx, hy] = this.toScreen(...h.get());
          if (Math.hypot(hx - px, hy - py) < (h.r || 18)) { this.drag = { h }; c.setPointerCapture(e.pointerId); e.preventDefault(); return; }
        }
        if (this.opts.onClick) { const [wx, wy] = this.toWorld(px, py); if (this.opts.onClick(wx, wy, e)) return; }
        if (this.opts.pan) { this.drag = { pan: [px, py, this.cx, this.cy] }; c.setPointerCapture(e.pointerId); }
      });
      c.addEventListener('pointermove', e => {
        const [px, py] = pos(e);
        if (!this.drag) {
          const over = this.handles.some(h => { const [hx, hy] = this.toScreen(...h.get()); return Math.hypot(hx - px, hy - py) < (h.r || 18); });
          c.style.cursor = over ? 'grab' : (this.opts.pan ? 'move' : 'default');
          return;
        }
        if (this.drag.h) { const [wx, wy] = this.toWorld(px, py); this.drag.h.set(wx, wy); }
        else { const [x0, y0, cx, cy] = this.drag.pan; this.cx = cx - (px - x0) / this.scale; this.cy = cy + (py - y0) / this.scale; this.userMoved = true; }
        this.onChange && this.onChange();
      });
      const up = () => { if (this.drag && this.drag.h && this.drag.h.end) this.drag.h.end(); this.drag = null; };
      c.addEventListener('pointerup', up); c.addEventListener('pointercancel', up);
      c.addEventListener('wheel', e => {
        if (!(e.ctrlKey || e.metaKey) || !this.opts.pan) return;
        e.preventDefault(); const [px, py] = pos(e); this.zoom(e.deltaY < 0 ? 1.15 : 1 / 1.15, px, py); this.userMoved = true; this.onChange && this.onChange();
      }, { passive: false });
    }
  }

  /* ------------------------------------------------------------ controls */
  function slider(host, { label, min, max, step, value, fmt = v => v, onInput, log = false, id }) {
    const wrap = document.createElement('label'); wrap.className = 'ctl';
    const top = document.createElement('span'); top.className = 'ctl-top';
    const name = document.createElement('span'); name.innerHTML = label;
    const out = document.createElement('output');
    top.append(name, out);
    const inp = document.createElement('input'); inp.type = 'range';
    if (id) inp.id = id;
    const toV = s => log ? Math.round(10 ** s * 1e6) / 1e6 : +s, toS = v => log ? Math.log10(v) : v;
    inp.min = toS(min); inp.max = toS(max); inp.step = log ? (toS(max) - toS(min)) / 100 : step; inp.value = toS(value);
    const upd = () => { const v = toV(inp.value); out.textContent = fmt(v); onInput && onInput(v); };
    inp.addEventListener('input', upd);
    wrap.append(top, inp); host.appendChild(wrap);
    out.textContent = fmt(value);
    return { get: () => toV(inp.value), set: v => { inp.value = toS(v); out.textContent = fmt(v); }, el: inp };
  }
  function select(host, { label, options, value, onChange }) {
    const wrap = document.createElement('label'); wrap.className = 'ctl';
    const top = document.createElement('span'); top.className = 'ctl-top'; top.innerHTML = `<span>${label}</span>`;
    const sel = document.createElement('select');
    options.forEach(([v, t]) => { const o = document.createElement('option'); o.value = v; o.textContent = t; sel.appendChild(o); });
    sel.value = value; sel.addEventListener('change', () => onChange && onChange(sel.value));
    wrap.append(top, sel); host.appendChild(wrap);
    return { get: () => sel.value, set: v => { sel.value = v; }, el: sel };
  }
  function toggle(host, { label, value, onChange }) {
    const wrap = document.createElement('label'); wrap.className = 'ctl ctl-check';
    const inp = document.createElement('input'); inp.type = 'checkbox'; inp.checked = value;
    inp.addEventListener('change', () => onChange && onChange(inp.checked));
    const s = document.createElement('span'); s.innerHTML = label;
    wrap.append(inp, s); host.appendChild(wrap);
    return { get: () => inp.checked, set: v => { inp.checked = v; }, el: inp };
  }
  function button(host, label, onClick, cls = '') {
    const b = document.createElement('button'); b.type = 'button'; b.className = 'btn ' + cls; b.innerHTML = label;
    b.addEventListener('click', onClick); host.appendChild(b); return b;
  }
  /* run callback only while element is on screen */
  function whenVisible(elm, onShow, onHide) {
    new IntersectionObserver(es => es.forEach(e => (e.isIntersecting ? onShow() : onHide && onHide())), { threshold: 0.05 }).observe(elm);
  }
  const fmtDeg = r => (r * 180 / Math.PI).toFixed(0) + '°';

  return { readTokens, T, onTheme, themeChanged, Viewport, slider, select, toggle, button, whenVisible, fmtDeg };
})();
