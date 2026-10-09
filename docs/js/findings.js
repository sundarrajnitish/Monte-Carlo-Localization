/* Findings: charts built from results.json (python/scripts/run_experiments.py). */
'use strict';

Widgets.findings = (grid, D) => {
  const R = D.results, T = () => UI.T();
  const card = (title, take, wide) => {
    const c = document.createElement('div'); c.className = 'finding' + (wide ? ' wide' : '');
    c.innerHTML = `<h3>${title}</h3><p class="take">${take}</p>`;
    grid.appendChild(c); return c;
  };
  const box = (c, h) => { const d = document.createElement('div'); if (h) d.style.minHeight = h + 'px'; c.appendChild(d); return d; };
  const renders = [];
  const reg = f => { renders.push(f); f(); };

  /* F1 real log --------------------------------------------------------- */
  const rt = R.real_tracking_summary, rs = R.real_series;
  const c1 = card('1 · On the real log, MCL keeps the scan on the walls; odometry alone does not',
    `Over ${(rt.path_length_cm / 100).toFixed(1)} m and ${rt.duration_s.toFixed(0)} s, integrating odometry from the correct start drifts to <b>${rt.dr_vs_lf_final_cm.toFixed(0)} cm</b> from the filter's final pose, mostly through heading drift. With MCL the median distance from a scan end-point to the nearest wall is <b>${rt.median_residual_cm.likelihood_field.toFixed(0)} cm</b> (one grid cell), against <b>${rt.median_residual_cm.dead_reckoning.toFixed(0)} cm</b> for odometry. The two sensor models agree to <b>${rt.lf_vs_beam_cm.mean.toFixed(1)} cm</b> on average (max ${rt.lf_vs_beam_cm.max.toFixed(1)} cm); the likelihood field is <b>${(rt.ms_per_scan.beam / rt.ms_per_scan.likelihood_field).toFixed(0)}×</b> cheaper.`, true);
  const mapWrap = document.createElement('div'); mapWrap.className = 'canvas-wrap'; mapWrap.innerHTML = '<canvas></canvas>'; c1.appendChild(mapWrap);
  const lg = document.createElement('div'); lg.className = 'legend-row';
  lg.innerHTML = '<span><i class="line" style="border-color:var(--series-1)"></i>MCL, likelihood field</span><span><i class="line" style="border-color:var(--series-2)"></i>MCL, beam model</span><span><i class="dash" style="border-color:var(--odom)"></i>odometry only</span>';
  c1.appendChild(lg);
  const vp = new UI.Viewport(mapWrap.querySelector('canvas'), D.map, { aspect: 4.2, minH: 200, maxH: 320 });
  const drawMap = () => {
    vp.fit([560, -10, 1960, D.map.height + 10]);
    const t = T(); vp.begin(); vp.drawMap();
    vp.path(D.reference.odom, t.odom, 2, [6, 5]); vp.path(D.reference.beam, t.s2, 2); vp.path(D.reference.lf, t.s1, 2);
    const s = D.reference.lf[0]; vp.pose(s[0], s[1], s[2], t.ink3, 12);
  };
  vp.onChange = drawMap; UI.onTheme(() => { vp.mapImg = null; drawMap(); }); drawMap();
  const ch1 = box(c1, 240);
  reg(() => Charts.line(ch1, [
    { name: 'MCL (likelihood field)', color: T().s1, data: rs.likelihood_field.residual.map((v, i) => [i, v]) },
    { name: 'odometry only', color: T().odom, data: rs.dead_reckoning.residual.map((v, i) => [i, v]), dash: true },
  ], { xLabel: 'scan', xName: 'scan', yLabel: 'median end-point to wall (cm)', yUnit: ' cm', height: 240 }));

  /* F2 global localization ------------------------------------------------ */
  const g = R.real_global, temps = [...new Set(g.map(r => r.temperature))], ns = [...new Set(g.map(r => r.n))];
  const best = g.reduce((a, b) => (b.success / b.runs > a.success / a.runs ? b : a));
  const c2 = card('2 · Global localization needs particles <em>and</em> humility',
    `Starting from particles spread uniformly over the whole map (${g[0].runs} seeds each). With 1000 particles the filter usually locks onto the wrong place early and never recovers. Tripling the particles helps most; softening the likelihood with temperature T helps a little more, because an over-confident sensor model kills the correct hypothesis before its evidence arrives. Best: <b>${best.success}/${best.runs}</b> at N = ${best.n}, T = ${best.temperature}. The failures are not noisy; they converge confidently onto a wrong stretch of corridor that happens to look the same, which is the aliasing you can see with the heat map in chapter 1.`);
  const ch2 = box(c2, 260);
  const scol = [() => T().s1, () => T().s2];
  reg(() => Charts.bars(ch2, temps.map(t => 'T = ' + t), ns.map((n, i) => ({
    name: `N = ${n}`, color: scol[i % 2](), values: temps.map(t => { const r = g.find(q => q.temperature === t && q.n === n); return 100 * r.success / r.runs; }),
  })), { yLabel: 'runs localized (%)', yMax: 100, labels: true, valFmt: v => v.toFixed(0) + '%', height: 260 }));

  /* F3 accuracy vs N -------------------------------------------------------- */
  const P = R.particles, ns3 = [...new Set(P.map(r => r.n))];
  const get = (m, key) => ns3.map(n => { const r = P.find(q => q.model === m && q.n === n); return [n, r[key]]; });
  const lf1000 = P.find(r => r.model === 'likelihood_field' && r.n === 1000), bm1000 = P.find(r => r.model === 'beam' && r.n === 1000);
  const failRows = P.filter(r => r.failures > 0);
  const failTxt = failRows.length ? ` With only ${Math.max(...failRows.map(r => r.n))} particles or fewer, some runs lose track entirely (${failRows.map(r => `${r.failures}/${r.runs} at N=${r.n}, ${r.model.replace('_', ' ')}`).join('; ')}).` : ' Even 25 particles never lost track in these runs: with a known start region, tracking is easy.';
  const c3 = card('3 · Accuracy saturates early; the beam model is more precise',
    `Tracking error (RMSE vs ground truth, synthetic, ${P[0].runs} runs each) against particle count. Past a few hundred particles the likelihood field plateaus near <b>${lf1000.rmse.toFixed(1)} cm</b>: its error is set by the sensor model, not by sampling. The beam model uses free space as evidence too and keeps improving (<b>${bm1000.rmse.toFixed(1)} cm</b> at 1000).${failTxt}`);
  const ch3 = box(c3, 250);
  reg(() => Charts.line(ch3, [
    { name: 'likelihood field', color: T().s1, data: get('likelihood_field', 'rmse'), points: true },
    { name: 'beam model', color: T().s2, data: get('beam', 'rmse'), points: true },
  ], { xLog: true, xLabel: 'particles (log scale)', xName: 'particles', yLabel: 'RMSE (cm)', yUnit: ' cm', height: 250, yMin: 0 }));

  /* F4 cost ----------------------------------------------------------------- */
  const c4 = card('4 · …and pays for it in compute',
    `Time per scan (Python/NumPy, one CPU core) grows linearly with particles for both, but every beam-model particle needs 36 ray casts. At 1000 particles: likelihood field <b>${lf1000.ms_per_scan.toFixed(1)} ms</b>, beam model <b>${bm1000.ms_per_scan.toFixed(1)} ms</b>. The robot produces ~5 scans per second, so both are real-time here; with a larger map or more beams, the likelihood field is the practical choice.`);
  const ch4 = box(c4, 250);
  reg(() => Charts.line(ch4, [
    { name: 'likelihood field', color: T().s1, data: get('likelihood_field', 'ms_per_scan'), points: true },
    { name: 'beam model', color: T().s2, data: get('beam', 'ms_per_scan'), points: true },
  ], { xLog: true, xLabel: 'particles (log scale)', xName: 'particles', yLabel: 'ms per scan', yUnit: ' ms', height: 250 }));

  /* F5 drift ------------------------------------------------------------------ */
  const dr = R.dead_reckoning_synthetic;
  const peak = Math.max(...dr.median_error);
  const c5 = card('5 · Odometry error grows to metres; MCL stays at centimetres',
    `Median over ${P[0].runs} synthetic runs of the ~33 m tour. Dead reckoning (odometry integrated from the true start) reaches <b>${(peak / 100).toFixed(1)} m</b> of error${peak > dr.final * 1.2 ? ` and ends ${(dr.final / 100).toFixed(1)} m off` : ' by the end of the tour'}: a small heading error, carried over metres of travel, becomes a large position error. MCL's RMSE is <b>${dr.mcl_rmse.toFixed(1)} cm</b>, and it is anisotropic: <b>${dr.rmse_along_corridor.toFixed(1)} cm</b> along the corridor vs <b>${dr.rmse_across_corridor.toFixed(1)} cm</b> across it, because the walls pin down only the sideways position. One caveat: the particle cloud's own spread (<b>${dr.mcl_mean_spread.toFixed(1)} cm</b> on average) is smaller than the actual error, so the filter is over-confident. That is the price of treating 36 correlated beams as independent.`);
  const ch5 = box(c5, 250);
  reg(() => Charts.line(ch5, [
    { name: 'odometry only', color: T().odom, data: dr.median_error.map((v, i) => [i, v]), dash: true },
    { name: 'MCL (likelihood field)', color: T().s1, data: dr.mcl_median_error.map((v, i) => [i, v]) },
  ], { xLabel: 'scan', xName: 'scan', yLabel: 'position error (cm)', yUnit: ' cm', height: 250 }));

  /* F6 kidnapped ----------------------------------------------------------- */
  const kd = R.kidnap, std = kd.curves['standard MCL'], aug = kd.curves['augmented MCL'];
  const recN = a => a.filter(v => v !== null).length, med = a => { const v = a.filter(x => x !== null).sort((p, q) => p - q); return v.length ? v[Math.floor(v.length / 2)] : null; };
  const c6 = card('6 · The kidnapped robot: only augmented MCL recovers',
    `At scan ${kd.kidnap_scan} the robot is picked up mid-corridor and put back in the start doorway; odometry sees nothing. Standard MCL has no particles left near the truth and recovered in <b>${recN(std.recovered_after_scans)}/${std.recovered_after_scans.length}</b> runs. Augmented MCL tracks a short- and a long-term average of the measurement likelihood and injects random particles when the short one drops: it recovered in <b>${recN(aug.recovered_after_scans)}/${aug.recovered_after_scans.length}</b> runs, after a median of ${med(aug.recovered_after_scans)} scans.`);
  const ch6 = box(c6, 250);
  reg(() => Charts.line(ch6, [
    { name: 'standard MCL', color: T().s2, data: std.median_error.map((v, i) => [i, v]) },
    { name: 'augmented MCL', color: T().s1, data: aug.median_error.map((v, i) => [i, v]) },
  ], { xLabel: 'scan', xName: 'scan', yLabel: 'median error (cm)', yUnit: ' cm', height: 250, markers: [{ x: kd.kidnap_scan, label: 'kidnap' }] }));

  /* F7 resamplers ------------------------------------------------------------ */
  const rsm = R.resamplers;
  const c7 = card('7 · Resampling scheme matters less than you think',
    `200 particles, ${rsm[0].runs} synthetic runs each. All schemes track fine; the differences in RMSE are within run-to-run noise at this size. The consistent effect is diversity: low-variance and stratified resampling keep a few more distinct particles than multinomial, which is exactly what the resampling widget shows.`);
  const tb = document.createElement('div'); tb.className = 'table-wrap';
  tb.innerHTML = `<table><thead><tr><th>scheme</th><th>when</th><th class="num">RMSE</th><th class="num">distinct after resample</th></tr></thead><tbody>` +
    rsm.map(r => `<tr><td>${r.resampler.replace('_', '-')}</td><td>${r.policy}</td><td class="num">${r.rmse.toFixed(1)} cm</td><td class="num">${r.unique_after_resample.toFixed(0)} / 200</td></tr>`).join('') + '</tbody></table>';
  c7.appendChild(tb);

  /* F8 odometry streams ----------------------------------------------------- */
  const os = rt.odometry_streams, ir = rt.interleaved_run;
  const c8 = card('8 · Picking the right odometry stream',
    `The log carries odometry twice: in <code>O</code> records and stamped on each <code>L</code> (laser) record. The stamped pose lags the odometry stream by about one reading. Merging both by timestamp would make the robot appear to shuffle back and forth: the path doubles and the motion model would be asked to explain hundreds of reversals that never happened. The filter therefore uses one stream, the laser-stamped pose, which is the pose each scan was actually taken at.`);
  const tb2 = document.createElement('div'); tb2.className = 'table-wrap';
  tb2.innerHTML = `<table><thead><tr><th>odometry used</th><th class="num">path length</th><th class="num">apparent reversals</th></tr></thead><tbody>` +
    [['O records only', os['O only']], ['L records only', os['L only']], ['both, merged by time', os.interleaved]].map(([n, v]) =>
      `<tr><td>${n}</td><td class="num">${(v.path_cm / 100).toFixed(1)} m</td><td class="num">${v.reversals}</td></tr>`).join('') + '</tbody></table>' +
    `<p class="small muted" style="margin:10px 0 0">On this log the scans still pull the filter back each step, so the damage is limited (mean particle spread ${ir.mean_spread.toFixed(1)} cm merged vs ${ir.mean_spread_laser.toFixed(1)} cm with one stream), but the motion model is being fed motion that never happened.</p>`;
  c8.appendChild(tb2);

  let rt2;
  const rerender = () => { clearTimeout(rt2); rt2 = setTimeout(() => renders.forEach(f => f()), 120); };
  UI.onTheme(rerender);
  let lastW = grid.clientWidth;
  new ResizeObserver(() => { if (Math.abs(grid.clientWidth - lastW) > 8) { lastW = grid.clientWidth; rerender(); } }).observe(grid);
};
