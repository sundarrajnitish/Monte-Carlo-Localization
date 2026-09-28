/* Boot: load data, theme, nav highlighting, then start every widget. */
'use strict';

(async () => {
  UI.readTokens();
  const toggle = document.getElementById('theme-toggle');
  toggle.addEventListener('click', () => {
    const dark = document.documentElement.dataset.theme
      ? document.documentElement.dataset.theme === 'dark'
      : matchMedia('(prefers-color-scheme: dark)').matches;
    document.documentElement.dataset.theme = dark ? 'light' : 'dark';
    try { localStorage.setItem('mcl-theme', document.documentElement.dataset.theme); } catch (e) { /* private mode */ }
    UI.themeChanged();
  });
  matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => UI.themeChanged());

  // highlight the chapter in view
  const links = [...document.querySelectorAll('.nav a')];
  const io = new IntersectionObserver(es => es.forEach(e => {
    if (e.isIntersecting) links.forEach(a => a.classList.toggle('active', a.getAttribute('href') === '#' + e.target.id));
  }), { rootMargin: '-40% 0px -55% 0px' });
  document.querySelectorAll('section.chapter').forEach(s => io.observe(s));

  let data, results;
  try {
    [data, results] = await Promise.all([fetch('data/dataset.json').then(r => r.json()), fetch('data/results.json').then(r => r.json())]);
  } catch (e) {
    document.querySelector('.hero .lede').insertAdjacentHTML('afterend', '<p class="callout">Could not load <code>data/*.json</code>. If you opened this file directly from disk, serve the folder instead: <code>python -m http.server -d docs</code>.</p>');
    return;
  }
  const D = {
    map: new MCL.GridMap(data.rows, data.resolution),
    scans: data.scans,
    reference: data.reference,
    startBox: data.start_box,
    results,
  };
  const safe = (name, fn) => { try { fn(); } catch (e) { console.error(name, e); } };
  safe('hero', () => Widgets.hero(document.getElementById('hero'), D));
  safe('manual', () => Widgets.manual(document.getElementById('w-manual'), D));
  safe('corridor', () => Widgets.corridor(document.getElementById('w-1d')));
  safe('motion', () => Widgets.motion(document.getElementById('w-motion')));
  safe('beam', () => Widgets.beam(document.getElementById('w-beam')));
  safe('field', () => Widgets.field(document.getElementById('w-field'), D));
  safe('resample', () => Widgets.resample(document.getElementById('w-resample')));
  safe('playground', () => Widgets.playground(document.getElementById('w-play'), D));
  safe('findings', () => Widgets.findings(document.getElementById('findings-grid'), D));
})();
