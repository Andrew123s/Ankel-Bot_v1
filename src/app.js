/* Ankel-Bot viewer: scene, view modes, build sequence, internal views and operating simulation. */
(function () {
  const T = THREE, AB = window.AB;
  const $ = (s) => document.querySelector(s);
  const view = $('#view'), canvas = $('#c'), labelsEl = $('#labels'), ctx = $('#ctx'), hud = $('#hud'), pip = $('#pip');
  const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
  const lerp = (a, b, t) => a + (b - a) * t;
  const ease = (t) => 1 - Math.pow(1 - t, 3);
  const rnd = (a, b) => a + Math.random() * (b - a);

  /* ---------- renderer / scene ---------- */
  const renderer = new T.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.shadowMap.enabled = true; renderer.shadowMap.type = T.PCFSoftShadowMap;
  renderer.toneMapping = T.ACESFilmicToneMapping; renderer.toneMappingExposure = 1.0;
  renderer.localClippingEnabled = true;
  const scene = new T.Scene();
  const camera = new T.PerspectiveCamera(36, 1, 0.05, 500); camera.layers.enable(1);

  const sky = (() => {
    const c = document.createElement('canvas'); c.width = 1024; c.height = 512; const g = c.getContext('2d');
    const gr = g.createLinearGradient(0, 0, 0, 512);
    gr.addColorStop(0, '#8ea6bf'); gr.addColorStop(0.4, '#c9d6df'); gr.addColorStop(0.498, '#e6ecee'); gr.addColorStop(0.502, '#77806f'); gr.addColorStop(1, '#3c453b');
    g.fillStyle = gr; g.fillRect(0, 0, 1024, 512);
    const s = g.createRadialGradient(720, 110, 0, 720, 110, 170); s.addColorStop(0, 'rgba(255,250,235,.95)'); s.addColorStop(1, 'rgba(255,250,235,0)');
    g.fillStyle = s; g.fillRect(0, 0, 1024, 512);
    const t = new T.CanvasTexture(c); t.colorSpace = T.SRGBColorSpace; t.mapping = T.EquirectangularReflectionMapping; return t;
  })();
  scene.environment = new T.PMREMGenerator(renderer).fromEquirectangular(sky).texture;
  const studioBg = new T.Color(0x20242a), fog = new T.Fog(0xd3dbde, 40, 220);

  scene.add(new T.HemisphereLight(0xe2ebf3, 0x3c4238, 0.9));
  const sun = new T.DirectionalLight(0xfff3e2, 2.4);
  sun.castShadow = true; sun.shadow.mapSize.set(2048, 2048); sun.shadow.bias = -0.0004; sun.shadow.normalBias = 0.01;
  Object.assign(sun.shadow.camera, { left: -3, right: 3, top: 3, bottom: -3, near: 1, far: 20 });
  scene.add(sun, sun.target);

  const bot = new T.Group(); bot.rotation.order = 'YXZ'; scene.add(bot); bot.add(AB.root);

  /* ---------- water ---------- */
  const TILE = 2.5;
  const waterN = (() => {
    const N = 256, c = document.createElement('canvas'); c.width = c.height = N; const g = c.getContext('2d'), img = g.createImageData(N, N), w = [];
    for (let i = 0; i < 22; i++) w.push([Math.round(rnd(-10, 10)), Math.round(rnd(-10, 10)), rnd(0, 6.28)]);
    const h = (x, y) => { let s = 0; for (const q of w) s += Math.sin(((q[0] * x + q[1] * y) * 2 * Math.PI) / N + q[2]) / (Math.hypot(q[0], q[1]) || 1); return s; };
    for (let y = 0; y < N; y++) for (let x = 0; x < N; x++) {
      const dx = (h(x + 1, y) - h(x - 1, y)) * 0.9, dy = (h(x, y + 1) - h(x, y - 1)) * 0.9, l = Math.hypot(dx, dy, 1), i = (y * N + x) * 4;
      img.data[i] = (-dx / l * 0.5 + 0.5) * 255; img.data[i + 1] = (-dy / l * 0.5 + 0.5) * 255; img.data[i + 2] = (1 / l * 0.5 + 0.5) * 255; img.data[i + 3] = 255;
    }
    g.putImageData(img, 0, 0);
    const t = new T.CanvasTexture(c); t.wrapS = t.wrapT = T.RepeatWrapping; t.repeat.set(400 / TILE, 400 / TILE); t.anisotropy = 8; return t;
  })();
  const waterGeo = new T.PlaneGeometry(400, 400); waterGeo.rotateX(-Math.PI / 2);
  const water = new T.Mesh(waterGeo, new T.MeshStandardMaterial({ color: 0x2c3b30, roughness: 0.1, metalness: 0, normalMap: waterN, normalScale: new T.Vector2(0.45, 0.45), transparent: true, opacity: 0.9 }));
  water.receiveShadow = true; water.renderOrder = 2; scene.add(water);

  /* ---------- studio floor ---------- */
  const studio = new T.Group(); scene.add(studio);
  const floorGeo = new T.CircleGeometry(14, 64); floorGeo.rotateX(-Math.PI / 2);
  const floor = new T.Mesh(floorGeo, new T.MeshStandardMaterial({ color: 0x2a2f36, roughness: 0.95 })); floor.position.y = -0.62; floor.receiveShadow = true; studio.add(floor);
  const grid = new T.GridHelper(10, 50, 0x4a525c, 0x363c44); grid.position.y = -0.618; studio.add(grid);

  /* ---------- wake ---------- */
  const streak = (() => {
    const c = document.createElement('canvas'); c.width = 64; c.height = 256; const g = c.getContext('2d');
    for (let i = 0; i < 260; i++) { g.fillStyle = 'rgba(255,255,255,' + rnd(0.05, 0.5) + ')'; g.fillRect(rnd(0, 64), rnd(0, 256), rnd(1, 5), rnd(6, 40)); }
    const t = new T.CanvasTexture(c); t.wrapS = t.wrapT = T.RepeatWrapping; t.colorSpace = T.SRGBColorSpace; return t;
  })();
  const fade = (() => {
    const c = document.createElement('canvas'); c.width = 64; c.height = 128; const g = c.getContext('2d');
    const gr = g.createLinearGradient(0, 128, 0, 0); gr.addColorStop(0, '#000'); gr.addColorStop(0.12, '#fff'); gr.addColorStop(1, '#000'); g.fillStyle = gr; g.fillRect(0, 0, 64, 128);
    const gx = g.createLinearGradient(0, 0, 64, 0); gx.addColorStop(0, '#000'); gx.addColorStop(0.3, 'rgba(0,0,0,0)'); gx.addColorStop(0.7, 'rgba(0,0,0,0)'); gx.addColorStop(1, '#000'); g.fillStyle = gx; g.fillRect(0, 0, 64, 128);
    return new T.CanvasTexture(c);
  })();
  const wakeMat = new T.MeshBasicMaterial({ map: streak, alphaMap: fade, transparent: true, depthWrite: false, opacity: 0 });
  const wakes = new T.Group(); bot.add(wakes);
  [[0.40, 0.34, -1.08], [-0.40, 0.34, -1.08], [0, 0.46, -1.8]].forEach((q) => {
    const g = new T.PlaneGeometry(q[1], 2.2); g.rotateX(-Math.PI / 2);
    const m = new T.Mesh(g, wakeMat); m.position.set(q[0], 0.006, q[2] - 1.1); m.renderOrder = 3; wakes.add(m);
  });

  /* ---------- orbit controls ---------- */
  const ctl = { t: new T.Vector3(0, 0.1, -0.2), r: 5, th: 0.85, ph: 1.12, gt: new T.Vector3(0, 0.1, -0.2), gr: 5, gth: 0.85, gph: 1.12 };
  function goView(th, ph, r, tx, ty, tz) { ctl.gth = th; ctl.gph = ph; if (r) ctl.gr = r; if (tx !== undefined) ctl.gt.set(tx, ty, tz); }
  (function () {
    const ptr = new Map(); let pd = 0;
    canvas.addEventListener('contextmenu', (e) => e.preventDefault());
    canvas.addEventListener('pointerdown', (e) => { canvas.setPointerCapture(e.pointerId); ptr.set(e.pointerId, { x: e.clientX, y: e.clientY, b: e.button, s: e.shiftKey }); });
    canvas.addEventListener('pointerup', (e) => { ptr.delete(e.pointerId); pd = 0; });
    canvas.addEventListener('pointercancel', (e) => { ptr.delete(e.pointerId); pd = 0; });
    canvas.addEventListener('pointermove', (e) => {
      const p = ptr.get(e.pointerId); if (!p) return;
      const dx = e.clientX - p.x, dy = e.clientY - p.y; p.x = e.clientX; p.y = e.clientY;
      if (ptr.size === 2) {
        const a = [...ptr.values()], d = Math.hypot(a[0].x - a[1].x, a[0].y - a[1].y);
        if (pd) ctl.gr = clamp(ctl.gr * (pd / d), 0.6, 40); pd = d; return;
      }
      if (p.b === 2 || p.s) {
        const k = ctl.r * 0.0011, right = new T.Vector3().setFromMatrixColumn(camera.matrix, 0), up = new T.Vector3().setFromMatrixColumn(camera.matrix, 1);
        ctl.gt.addScaledVector(right, -dx * k).addScaledVector(up, dy * k);
      } else { ctl.gth -= dx * 0.006; ctl.gph = clamp(ctl.gph - dy * 0.006, 0.05, 1.62); }
    });
    canvas.addEventListener('wheel', (e) => { e.preventDefault(); ctl.gr = clamp(ctl.gr * Math.exp(e.deltaY * 0.0011), 0.6, 40); }, { passive: false });
  })();
  function updateCamera(dt) {
    const k = Math.min(1, dt * 9);
    ctl.t.lerp(ctl.gt, k); ctl.r = lerp(ctl.r, ctl.gr, k); ctl.th = lerp(ctl.th, ctl.gth, k); ctl.ph = lerp(ctl.ph, ctl.gph, k);
    const s = Math.sin(ctl.ph);
    camera.position.set(ctl.t.x + ctl.r * s * Math.sin(ctl.th), ctl.t.y + ctl.r * Math.cos(ctl.ph), ctl.t.z + ctl.r * s * Math.cos(ctl.th));
    camera.lookAt(ctl.t);
  }

  /* ---------- state ---------- */
  const S = { mode: 'assembled', sel: 0, water: true, dims: false, explode: 1, xray: true, cut: 0, labels: true, stepP: 9, stepT: 9, playing: false, playT: 0, overlay: true, paused: false };
  const HIDE_INTERNAL = new Set(['aftDeck', 'lids', 'encLid', 'fasteners']);
  const byCat = {}; AB.list.forEach((p) => { (byCat[p.cat] = byCat[p.cat] || []).push(p); });
  const clipPlane = new T.Plane(new T.Vector3(-1, 0, 0), 1);

  function applyShell() {
    const x = S.mode === 'internal' && S.xray, c = S.mode === 'internal' && S.cut > 0.01;
    AB.shellMats.forEach((m) => { m.transparent = x; m.opacity = x ? 0.22 : 1; m.depthWrite = !x; m.needsUpdate = true; });
    clipPlane.constant = 0.56 - S.cut * 0.66;
    AB.clipMats.forEach((m) => { m.clippingPlanes = c ? [clipPlane] : null; m.side = c ? T.DoubleSide : T.FrontSide; m.needsUpdate = true; });
  }

  /* ---------- labels ---------- */
  const mkLabel = (txt, cls) => { const d = document.createElement('div'); d.className = 'label' + (cls ? ' ' + cls : ''); d.textContent = txt; d.style.display = 'none'; labelsEl.appendChild(d); return d; };
  AB.list.forEach((p) => (p.el = mkLabel(p.label.split(' — ')[0])));
  const _v = new T.Vector3();
  function place(el, world, show) {
    if (!show) { el.style.display = 'none'; return; }
    _v.copy(world).project(camera);
    if (_v.z > 1 || Math.abs(_v.x) > 1.05 || Math.abs(_v.y) > 1.05) { el.style.display = 'none'; return; }
    el.style.display = ''; el.style.left = ((_v.x + 1) / 2) * view.clientWidth + 'px'; el.style.top = ((1 - _v.y) / 2) * view.clientHeight + 'px';
  }

  /* ---------- dimension lines ---------- */
  const dims = new T.Group(); scene.add(dims);
  const dimDefs = [
    { a: [0.68, -0.15, -0.79], b: [0.68, -0.15, 0.83], t: 'LOA 1.62 m' },
    { a: [-0.535, -0.15, -1.2], b: [0.535, -0.15, -1.2], t: 'Beam 1.07 m' },
    { a: [-0.68, -0.15, 0.6], b: [-0.68, 0.45, 0.6], t: 'Keel–camera 0.60 m' },
    { a: [0, 0.02, -0.83], b: [0, 0.02, -1.78], t: 'Net 0.95 m' },
  ];
  {
    const pts = [];
    dimDefs.forEach((d) => {
      const a = new T.Vector3(...d.a), b = new T.Vector3(...d.b), n = new T.Vector3().subVectors(b, a).normalize();
      const tick = Math.abs(n.y) > 0.5 ? new T.Vector3(0.05, 0, 0) : new T.Vector3(0, 0.05, 0);
      pts.push(a, b, a.clone().add(tick), a.clone().sub(tick), b.clone().add(tick), b.clone().sub(tick));
      d.mid = a.clone().lerp(b, 0.5); d.el = mkLabel(d.t, 'dim');
    });
    dims.add(new T.LineSegments(new T.BufferGeometry().setFromPoints(pts), new T.LineBasicMaterial({ color: 0xf05a1a })));
  }

  /* ---------- debris ---------- */
  const std = (o) => new T.MeshStandardMaterial(o);
  const DM = {
    pet: new T.MeshPhysicalMaterial({ color: 0xdce8f0, roughness: 0.08, transparent: true, opacity: 0.55 }), cap: std({ color: 0x2a6bd1, roughness: 0.5 }),
    can: std({ color: 0xb9302a, metalness: 0.7, roughness: 0.35 }), bag: std({ color: 0xeeeeea, roughness: 0.85, flatShading: true }),
    bagR: std({ color: 0xc94a4a, roughness: 0.85, flatShading: true }), leaf: std({ color: 0x7a5a2e, roughness: 0.95, side: T.DoubleSide }), cup: std({ color: 0xf4f4f0, roughness: 0.9 }),
  };
  const TYPES = [['bottle', 'PET bottle'], ['bottle', 'PET bottle'], ['bag', 'plastic bag'], ['can', 'drink can'], ['leaf', 'leaf litter'], ['cup', 'foam cup']];
  function debrisMesh(type) {
    const g = new T.Group(); let m;
    const add = (geo, mat) => { m = new T.Mesh(geo, mat); m.castShadow = true; g.add(m); return m; };
    if (type === 'bottle') {
      const b = new T.CylinderGeometry(0.032, 0.032, 0.17, 14); b.rotateZ(Math.PI / 2); add(b, DM.pet);
      const n = new T.CylinderGeometry(0.013, 0.03, 0.04, 12); n.rotateZ(-Math.PI / 2); add(n, DM.pet).position.x = 0.105;
      const c = new T.CylinderGeometry(0.015, 0.015, 0.018, 12); c.rotateZ(Math.PI / 2); add(c, DM.cap).position.x = 0.132;
    } else if (type === 'can') {
      const b = new T.CylinderGeometry(0.033, 0.033, 0.12, 16); b.rotateZ(Math.PI / 2); add(b, DM.can);
    } else if (type === 'bag') {
      const b = new T.IcosahedronGeometry(0.1, 1), p = b.attributes.position;
      for (let i = 0; i < p.count; i++) p.setXYZ(i, p.getX(i) * rnd(0.8, 1.2), p.getY(i) * rnd(0.6, 1.3), p.getZ(i) * rnd(0.8, 1.2));
      b.computeVertexNormals(); add(b, Math.random() < 0.4 ? DM.bagR : DM.bag).scale.set(1.2, 0.3, 0.9);
    } else if (type === 'leaf') {
      for (let i = 0; i < 4; i++) { const l = new T.CircleGeometry(0.035, 8); l.rotateX(-Math.PI / 2); const q = add(l, DM.leaf); q.scale.set(1.6, 1, 0.8); q.position.set(rnd(-0.07, 0.07), 0.012 + i * 0.002, rnd(-0.07, 0.07)); q.rotation.y = rnd(0, 6); q.castShadow = false; }
    } else { add(new T.CylinderGeometry(0.04, 0.028, 0.09, 14), DM.cup).rotation.z = 1.2; }
    return g;
  }
  const debris = [];
  for (let i = 0; i < 16; i++) { const t = TYPES[i % TYPES.length]; const d = { type: t[0], name: t[1], mesh: debrisMesh(t[0]), state: 'free', x: 0, z: 0, lx: 0, lz: 0, ph: rnd(0, 6), conf: rnd(0.8, 0.97), sx: 0, sz: 0 }; scene.add(d.mesh); debris.push(d); }

  /* ---------- AI overlay objects (layer 1 = not seen by the on-board camera) ---------- */
  const botCam = new T.PerspectiveCamera(50, 16 / 9, 0.05, 80);
  const HF = 0.66; // half horizontal FOV used by the detector, rad
  const wedge = new T.Group(); bot.add(wedge);
  {
    const sh = new T.Shape(), n = 16, r0 = 0.5, r1 = 6.5;
    for (let i = 0; i <= n; i++) { const a = -HF + (2 * HF * i) / n; (i ? sh.lineTo.bind(sh) : sh.moveTo.bind(sh))(Math.sin(a) * r1, Math.cos(a) * r1); }
    for (let i = n; i >= 0; i--) { const a = -HF + (2 * HF * i) / n; sh.lineTo(Math.sin(a) * r0, Math.cos(a) * r0); }
    const g = new T.ShapeGeometry(sh); g.rotateX(Math.PI / 2);
    const m = new T.Mesh(g, new T.MeshBasicMaterial({ color: 0x39d98a, transparent: true, opacity: 0.07, depthWrite: false, side: T.DoubleSide }));
    m.position.set(0, 0.02, 0.6); m.layers.set(1); m.renderOrder = 4; wedge.add(m);
    const e = new T.LineSegments(new T.BufferGeometry().setFromPoints([[-1, 0], [1, 0]].flatMap((q) => [new T.Vector3(0, 0.45, 0.6), new T.Vector3(q[0] * Math.sin(HF) * r1, 0.02, 0.6 + Math.cos(HF) * r1)])), new T.LineBasicMaterial({ color: 0x39d98a, transparent: true, opacity: 0.5 }));
    e.layers.set(1); wedge.add(e);
  }
  const ringGeo = new T.BufferGeometry().setFromPoints([[-1, -1], [1, -1], [1, 1], [-1, 1]].map((q) => new T.Vector3(q[0] * 0.17, 0, q[1] * 0.17)));
  const ringG = new T.LineBasicMaterial({ color: 0x39d98a }), ringT = new T.LineBasicMaterial({ color: 0xffb020 });
  const rings = debris.map(() => { const l = new T.LineLoop(ringGeo, ringG); l.layers.set(1); l.visible = false; scene.add(l); return l; });
  const aimGeo = new T.BufferGeometry().setFromPoints([new T.Vector3(), new T.Vector3()]);
  const aim = new T.Line(aimGeo, new T.LineBasicMaterial({ color: 0xffb020 })); aim.layers.set(1); aim.frustumCulled = false; scene.add(aim);
  const dets = debris.map(() => { const d = document.createElement('div'); d.className = 'det'; d.innerHTML = '<span></span>'; d.style.display = 'none'; pip.appendChild(d); return d; });

  /* ---------- simulation ---------- */
  const sim = { x: 0, z: 0, yaw: 0, v: 0, tP: 0, tS: 0, state: 'SEARCH', target: null, collected: 0, inNet: [], swap: 0, swapDir: 0, soc: 100, power: 0, flap: 1.1, t: 0, swaps: 0 };
  const CAP = 14;
  function spawn(d, first) {
    const a = sim.yaw + rnd(-0.8, 0.8), r = first ? rnd(2.5, 10) : rnd(7, 11);
    d.x = sim.x + Math.sin(a) * r; d.z = sim.z + Math.cos(a) * r; d.state = 'free'; d.conf = rnd(0.8, 0.97);
    scene.add(d.mesh); d.mesh.rotation.set(0, rnd(0, 6.28), 0); d.mesh.scale.setScalar(1);
  }
  debris.forEach((d) => spawn(d, true));
  function netHalfW(z) { return 0.25 - 0.08 * clamp((-0.83 - z) / 0.95, 0, 1); }
  function startSwap() { if (!sim.swapDir && sim.swap === 0) sim.swapDir = 1; }

  function step(dt) {
    sim.t += dt;
    const cs = Math.cos(sim.yaw), sn = Math.sin(sim.yaw), a = AB.wingAngle, reach = 0.81 + 0.8 * Math.cos(a);
    // --- perception: what the on-board camera can see ---
    let best = null, bd = 1e9;
    debris.forEach((d) => {
      d.seen = false; if (d.state !== 'free') return;
      const dx = d.x - sim.x, dz = d.z - sim.z; d.lx = dx * cs - dz * sn; d.lz = dx * sn + dz * cs; d.dist = Math.hypot(dx, dz); d.brg = Math.atan2(d.lx, d.lz - 0.6);
      if (d.lz > 0.95 && d.dist < 7 && Math.abs(d.brg) < HF) { d.seen = true; if (d.dist < bd) { bd = d.dist; best = d; } }
    });
    if (!sim.target || !sim.target.seen) sim.target = best;
    // --- control: bearing error -> differential thrust ---
    let yawRate, vT;
    if (sim.swapDir || sim.swap > 0) { sim.state = 'NET SWAP'; yawRate = 0; vT = 0; }
    else if (sim.target) { const b = Math.atan2(sim.target.lx, sim.target.lz); sim.state = 'TRACK'; yawRate = clamp(1.8 * b, -0.6, 0.6); vT = Math.abs(b) < 0.5 ? 0.5 : 0.28; }
    else { sim.state = 'SEARCH'; yawRate = 0.22; vT = 0.3; }
    sim.v = lerp(sim.v, vT, Math.min(1, dt * 1.5));
    const base = sim.v / 1.5, diff = yawRate * 0.45;
    sim.tS = lerp(sim.tS, clamp(base + diff, -1, 1), Math.min(1, dt * 4));
    sim.tP = lerp(sim.tP, clamp(base - diff, -1, 1), Math.min(1, dt * 4));
    sim.yaw += (sim.tS - sim.tP) / 0.9 * dt; sim.x += Math.sin(sim.yaw) * sim.v * dt; sim.z += Math.cos(sim.yaw) * sim.v * dt;
    sim.power = 30 + 350 * (Math.pow(Math.abs(sim.tP), 1.5) + Math.pow(Math.abs(sim.tS), 1.5));
    sim.soc = Math.max(0, sim.soc - (sim.power * dt) / 3600 / 2048 * 100);
    // --- debris ---
    let gateBusy = false;
    debris.forEach((d) => {
      const bobY = -0.012 + 0.006 * Math.sin(sim.t * 1.6 + d.ph);
      if (d.state === 'free') {
        if (d.lz > 0.6 && d.lz < reach - 0.04 && Math.abs(d.lx) < 0.27 + Math.max(0, d.lz - 0.81) * Math.tan(a)) {
          d.state = 'funnel'; bot.attach(d.mesh); d.mesh.position.set(d.lx, bobY, d.lz); d.sz = rnd(-1.68, -0.98); d.sx = rnd(-1, 1);
          if (sim.target === d) sim.target = null; return;
        }
        if (Math.abs(d.lx) < 0.62 && d.lz < 0.85 && d.lz > -1.9) { // slides along the hull side
          const nx = Math.sign(d.lx || 1) * 0.62; d.x = sim.x + nx * cs + d.lz * sn; d.z = sim.z - nx * sn + d.lz * cs;
        }
        if (d.lz < -3 || d.dist > 13) spawn(d, false);
        d.mesh.position.set(d.x, bobY, d.z); d.mesh.rotation.y += dt * 0.1;
      } else if (d.state === 'funnel') {
        const p = d.mesh.position; p.z -= Math.max(0.22, sim.v + 0.1) * dt; p.y = bobY;
        const lim = p.z > 0.81 ? 0.26 + (p.z - 0.81) * Math.tan(a) : p.z > -0.83 ? 0.16 : netHalfW(p.z) - 0.06;
        p.x = clamp(p.x, -lim, lim); d.mesh.rotation.y += dt * 0.8;
        if (p.z < 0.5 && p.z > -0.3) gateBusy = true;
        if (p.z <= d.sz) { d.state = 'net'; AB.netGroup.attach(d.mesh); sim.inNet.push(d); sim.collected++; d.mesh.position.x = d.sx * (netHalfW(d.sz) - 0.06); }
      } else if (d.state === 'net') { d.mesh.position.y = bobY - 0.005; }
    });
    if (sim.inNet.length >= CAP) startSwap();
    sim.flap = lerp(sim.flap, gateBusy ? 0.22 : 1.05 - 0.25 * clamp(sim.v, 0, 0.5), Math.min(1, dt * 5));
    // --- detachable net swap ---
    if (sim.swapDir) {
      sim.swap = clamp(sim.swap + sim.swapDir * dt / 2.2, 0, 1.25);
      if (sim.swapDir > 0 && sim.swap >= 1.25) {
        sim.inNet.forEach((d) => spawn(d, false)); sim.inNet.length = 0; sim.swaps++; sim.swapDir = -1; sim.swap = 1;
      } else if (sim.swapDir < 0 && sim.swap <= 0) sim.swapDir = 0;
    }
  }

  /* ---------- UI ---------- */
  const catsEl = $('#cats'), info = $('#info');
  AB.cats.forEach((c) => {
    const li = document.createElement('li'); li.innerHTML = '<b>' + c.n + '</b><span>' + c.title + '</span>'; li.onclick = () => select(c.n); catsEl.appendChild(li); c.li = li;
  });
  function select(n) {
    if (S.mode === 'assembly') { S.stepT = n; S.playing = false; S.sel = n; }
    else S.sel = S.sel === n ? 0 : n;
    renderInfo(); renderCtx();
  }
  function renderInfo() {
    AB.cats.forEach((c) => { c.li.classList.toggle('on', c.n === S.sel); c.li.classList.toggle('done', S.mode === 'assembly' && c.n < S.sel); });
    const c = AB.cats[S.sel - 1];
    if (!c) {
      info.innerHTML = '<h2>Overview</h2><p>Twin-hull autonomous surface vessel that funnels floating waste through a central tunnel into a detachable rear net, steered by an on-board vision model.</p>' +
        '<ul><li>1.62 × 1.07 × 0.60 m (hull) · ≈ 62 kg dry</li><li>Collection swath 1.15–2.0 m</li><li>Rear net ≈ 70 L, tool-free swap</li><li>2 × 350 W thrusters · 2 kWh LiFePO4</li></ul>' +
        '<p class="parts">Select a subsystem to isolate it. Drag to orbit, scroll to zoom, right-drag to pan. Figures are design targets for the prototype.</p>'; return;
    }
    info.innerHTML = '<h2>' + c.n + '. ' + c.title + '</h2><p>' + c.text + '</p><ul>' + c.specs.map((s) => '<li>' + s + '</li>').join('') + '</ul>' +
      '<p class="parts"><b>Parts:</b> ' + byCat[c.n].map((p) => p.label).join(' · ') + '</p>' + (S.mode === 'assembly' ? '' : '<button id="showAll">Show all</button>');
    const b = $('#showAll'); if (b) b.onclick = () => select(S.sel);
  }
  document.querySelectorAll('#modes .tab').forEach((b) => (b.onclick = () => setMode(b.dataset.m)));
  const VIEWS = { iso: [0.85, 1.12], top: [0.0001, 0.06], side: [Math.PI / 2, 1.5], front: [0, 1.45], rear: [Math.PI, 1.3] };
  document.querySelectorAll('#views button').forEach((b) => (b.onclick = () => { const v = VIEWS[b.dataset.v]; goView((S.mode === 'operation' ? sim.yaw : 0) + v[0], v[1]); }));

  function renderCtx() {
    const m = S.mode; let h = '';
    if (m === 'assembled') h = '<label><input type="checkbox" id="oWater"' + (S.water ? ' checked' : '') + '>On water</label><label><input type="checkbox" id="oDims"' + (S.dims ? ' checked' : '') + '>Dimensions</label>' +
      '<label>Wing sweep <input type="range" id="oWing" min="20" max="60" value="' + Math.round(AB.wingAngle * 57.3) + '"><span id="oWingV"></span></label><button id="oSwap">Detach / refit net</button>';
    if (m === 'exploded') h = '<label>Explode <input type="range" id="oExp" min="0" max="100" value="' + S.explode * 100 + '"></label><label><input type="checkbox" id="oLab"' + (S.labels ? ' checked' : '') + '>Labels</label>';
    if (m === 'internal') h = '<label><input type="checkbox" id="oXray"' + (S.xray ? ' checked' : '') + '>X-ray shells</label><label>Section cut <input type="range" id="oCut" min="0" max="100" value="' + S.cut * 100 + '"></label><label><input type="checkbox" id="oLab"' + (S.labels ? ' checked' : '') + '>Labels</label><span style="color:var(--dim)">Deck covers and lids removed</span>';
    if (m === 'assembly') { const c = AB.cats[clamp(S.stepT, 1, 9) - 1]; h = '<button id="aPrev">◀</button><button id="aPlay">' + (S.playing ? 'Pause' : 'Play') + '</button><button id="aNext">▶</button><div class="cap"><b>Step ' + S.stepT + ' / 9 — ' + (S.stepT ? c.title : 'Start') + '</b><span>' + (S.stepT ? c.step : 'Press Play to build the machine part by part.') + '</span></div>'; }
    if (m === 'operation') h = '<button id="pPause">' + (S.paused ? 'Resume' : 'Pause') + '</button><button id="oSwap">Swap net now</button><label><input type="checkbox" id="oOv"' + (S.overlay ? ' checked' : '') + '>AI overlay</label><label>Wing sweep <input type="range" id="oWing" min="20" max="60" value="' + Math.round(AB.wingAngle * 57.3) + '"><span id="oWingV"></span></label>';
    ctx.innerHTML = h;
    const on = (id, ev, f) => { const e = $('#' + id); if (e) e[ev] = f; };
    on('oWater', 'onchange', (e) => { S.water = e.target.checked; applyEnv(); });
    on('oDims', 'onchange', (e) => { S.dims = e.target.checked; });
    on('oLab', 'onchange', (e) => { S.labels = e.target.checked; });
    on('oExp', 'oninput', (e) => { S.explode = e.target.value / 100; });
    on('oXray', 'onchange', (e) => { S.xray = e.target.checked; applyShell(); });
    on('oCut', 'oninput', (e) => { S.cut = e.target.value / 100; applyShell(); });
    on('oOv', 'onchange', (e) => { S.overlay = e.target.checked; });
    on('pPause', 'onclick', () => { S.paused = !S.paused; renderCtx(); });
    on('oSwap', 'onclick', startSwap);
    const wv = $('#oWingV'); if (wv) wv.textContent = Math.round(AB.wingAngle * 57.3) + '°';
    on('oWing', 'oninput', (e) => { AB.setWings(e.target.value / 57.3); wv.textContent = e.target.value + '°'; });
    on('aPrev', 'onclick', () => { S.playing = false; S.stepT = Math.max(0, S.stepT - 1); S.sel = S.stepT; renderInfo(); renderCtx(); });
    on('aNext', 'onclick', () => { S.playing = false; S.stepT = Math.min(9, S.stepT + 1); S.sel = S.stepT; renderInfo(); renderCtx(); });
    on('aPlay', 'onclick', () => { S.playing = !S.playing; if (S.playing && S.stepT >= 9) { S.stepT = 0; S.stepP = 0; } S.playT = 0.6; renderCtx(); });
  }
  function applyEnv() {
    const w = S.mode === 'operation' || (S.mode === 'assembled' && S.water);
    water.visible = w; studio.visible = !w; scene.background = w ? sky : studioBg; scene.fog = w ? fog : null;
  }
  function setMode(m) {
    S.mode = m; S.sel = 0;
    document.querySelectorAll('#modes .tab').forEach((b) => b.classList.toggle('on', b.dataset.m === m));
    const op = m === 'operation';
    hud.style.display = pip.style.display = op ? 'block' : 'none';
    if (m === 'assembly') { S.stepP = 0; S.stepT = 0; S.playing = true; S.playT = 0.8; }
    if (m === 'assembled') goView(0.85, 1.12, 5.6, 0, 0.1, -0.3);
    if (m === 'assembly') goView(0.75, 1.0, 6.4, 0, 0.25, -0.3);
    if (m === 'exploded') goView(0.95, 1.05, 8.2, 0, 0.45, -0.5);
    if (m === 'internal') goView(0.55, 0.72, 4.4, 0, 0.05, -0.25);
    if (op) goView(sim.yaw + 2.5, 0.98, 7.5, sim.x, 0.1, sim.z);
    if (!op && sim.swapDir === 0) sim.swap = 0;
    applyEnv(); applyShell(); renderInfo(); renderCtx();
  }

  /* ---------- frame loop ---------- */
  const _w = new T.Vector3(), _a = new T.Vector3(), _b = new T.Vector3();
  let last = performance.now(), W = 0, H = 0, hudT = 0;
  function resize() {
    const w = view.clientWidth, h = view.clientHeight;
    if (w === W && h === H) return; W = w; H = h;
    renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix();
  }
  function frame(now) {
    requestAnimationFrame(frame);
    const dt = Math.min(0.05, (now - last) / 1000); last = now; resize();
    const m = S.mode, op = m === 'operation', tt = now / 1000;

    // simulation / pose
    if (op) {
      const px = sim.x, pz = sim.z;
      if (!S.paused) step(dt);
      ctl.gt.x += sim.x - px; ctl.gt.z += sim.z - pz; ctl.t.x += sim.x - px; ctl.t.z += sim.z - pz;
      bot.position.set(sim.x, 0.005 * Math.sin(tt * 1.7), sim.z);
      bot.rotation.set(0.012 * Math.sin(tt * 1.3), sim.yaw, 0.014 * Math.sin(tt * 1.1 + 1));
    } else {
      bot.position.set(0, water.visible ? 0.005 * Math.sin(tt * 1.7) : 0, 0); bot.rotation.set(0, 0, 0);
      if (sim.swapDir) { sim.swap = clamp(sim.swap + sim.swapDir * dt / 2.2, 0, 1.25); if (sim.swap >= 1.25) sim.swapDir = -1; else if (sim.swap <= 0) sim.swapDir = 0; }
    }
    const spin = op && !S.paused;
    AB.props.forEach((p) => { if (spin) p.obj.rotation.z += p.sg * (p.sg > 0 ? sim.tP : sim.tS) * 70 * dt; });
    AB.setFlaps(op ? sim.flap + 0.03 * Math.sin(tt * 5) : 1.05);
    wakeMat.opacity = op ? clamp((Math.abs(sim.tP) + Math.abs(sim.tS)) * 0.9, 0, 0.75) : 0; wakes.visible = op;
    if (spin) streak.offset.y += dt * 0.5;

    // build sequence
    if (m === 'assembly') {
      if (S.playing) { S.playT -= dt; if (S.playT <= 0 && Math.abs(S.stepP - S.stepT) < 0.02) { if (S.stepT < 9) { S.stepT++; S.sel = S.stepT; S.playT = 2.3; renderInfo(); renderCtx(); } else { S.playing = false; renderCtx(); } } }
      const d = S.stepT - S.stepP; S.stepP += Math.sign(d) * Math.min(Math.abs(d), dt / 1.3);
    }

    // per-part placement
    const sw = Math.min(1, sim.swap);
    AB.list.forEach((p) => {
      let vis = true, k = 0;
      if (m === 'exploded') k = S.explode;
      if (m === 'internal' && HIDE_INTERNAL.has(p.id)) vis = false;
      if (m === 'assembly') {
        const grp = byCat[p.cat], i = grp.indexOf(p), n = grp.length, t = clamp(S.stepP - (p.cat - 1), 0, 1);
        const q = clamp(t * (1 + 0.3 * (n - 1)) - 0.3 * i, 0, 1); vis = q > 0.001; p.k = (1 - ease(q)) * 1.5;
      } else p.k = lerp(p.k, k, Math.min(1, dt * 6));
      if (S.sel && m !== 'assembly' && m !== 'operation' && p.cat !== S.sel) vis = false;
      p.group.visible = vis; p.group.position.copy(p.explode).multiplyScalar(p.k);
      if (p.id === 'net') { p.group.position.y += 0.62 * clamp(sw * 1.6, 0, 1); p.group.position.z -= 0.35 * clamp((sw - 0.55) / 0.45, 0, 1); }
      const showL = S.labels && vis && ((m === 'exploded' && p.k > 0.35) || (m === 'internal' && (p.internal || p.id === 'net' || p.id === 'gate' || p.id === 'frame')));
      if (showL || p.el.style.display !== 'none') place(p.el, p.group.localToWorld(_w.copy(p.anchor)), showL);
    });

    // environment follows the boat
    water.position.set(bot.position.x, 0, bot.position.z);
    waterN.offset.set(bot.position.x / TILE + tt * 0.012, -bot.position.z / TILE + tt * 0.008);
    sun.position.set(bot.position.x + 3, 6.5, bot.position.z + 2.5); sun.target.position.copy(bot.position);
    updateCamera(dt);

    // dimensions
    const showD = m === 'assembled' && S.dims && !S.sel; dims.visible = showD;
    dimDefs.forEach((d) => place(d.el, d.mid, showD));

    // AI overlay + on-board camera
    debris.forEach((d) => (d.mesh.visible = op));
    wedge.visible = op && S.overlay; aim.visible = false;
    if (op) {
      bot.updateMatrixWorld(true);
      AB.camAnchor.getWorldPosition(_a); botCam.position.copy(_a); botCam.lookAt(bot.localToWorld(_b.set(0, -0.25, 3.4)));
      botCam.updateMatrixWorld(true);
      const pw = pip.clientWidth, ph = pip.clientHeight, f = ph / 2 / Math.tan((botCam.fov * Math.PI) / 360);
      debris.forEach((d, i) => {
        const r = rings[i], e = dets[i], seen = d.state === 'free' && d.seen, tg = d === sim.target;
        r.visible = seen && S.overlay; e.style.display = 'none';
        if (!seen) return;
        r.position.set(d.x, 0.03, d.z); r.material = tg ? ringT : ringG; r.rotation.y = sim.yaw;
        if (tg && S.overlay) { const a = aimGeo.attributes.position; a.setXYZ(0, _a.x, _a.y, _a.z); a.setXYZ(1, d.x, 0.03, d.z); a.needsUpdate = true; aim.visible = true; }
        _v.set(d.x, 0.02, d.z).project(botCam); if (_v.z > 1 || Math.abs(_v.x) > 0.98 || Math.abs(_v.y) > 0.98) return;
        const s = clamp((0.3 / d.dist) * f, 10, 90), cx = ((_v.x + 1) / 2) * pw, cy = ((1 - _v.y) / 2) * ph;
        e.style.display = ''; e.className = 'det' + (tg ? ' tgt' : ''); e.style.left = cx - s / 2 + 'px'; e.style.top = cy - s * 0.35 + 'px'; e.style.width = s + 'px'; e.style.height = s * 0.6 + 'px';
        e.firstChild.textContent = d.name + ' ' + clamp(d.conf - d.dist * 0.02, 0.4, 0.99).toFixed(2);
      });
      hudT -= dt;
      if (hudT <= 0) {
        hudT = 0.15; const fill = sim.inNet.length / CAP, pc = (v) => (v >= 0 ? '+' : '') + Math.round(v * 100) + '%';
        hud.innerHTML = '<div class="st">' + sim.state + (sim.target && sim.state === 'TRACK' ? ' · ' + sim.target.name : '') + '</div>' +
          '<div class="row"><span>Thrust port / stbd</span><b>' + pc(sim.tP) + ' / ' + pc(sim.tS) + '</b></div>' +
          '<div class="row"><span>Speed</span><b>' + sim.v.toFixed(2) + ' m/s</b></div>' +
          '<div class="row"><span>Heading</span><b>' + String(Math.round((((sim.yaw * 57.2958) % 360) + 360) % 360)).padStart(3, '0') + '°</b></div>' +
          '<div class="row"><span>Detections in view</span><b>' + debris.filter((d) => d.state === 'free' && d.seen).length + '</b></div>' +
          '<div class="row"><span>Items collected</span><b>' + sim.collected + '</b></div>' +
          '<div class="row"><span>Net fill</span><b>' + Math.round(fill * 100) + '% · swaps ' + sim.swaps + '</b></div><div class="bar"><i style="width:' + fill * 100 + '%"></i></div>' +
          '<div class="row"><span>Power draw</span><b>' + Math.round(sim.power) + ' W</b></div>' +
          '<div class="row"><span>Battery</span><b>' + sim.soc.toFixed(1) + '%</b></div>';
      }
    } else rings.forEach((r) => (r.visible = false));

    // render
    renderer.setScissorTest(false); renderer.setViewport(0, 0, W, H); renderer.render(scene, camera);
    if (op) {
      const r = pip.getBoundingClientRect(), vr = view.getBoundingClientRect(), x = r.left - vr.left, y = vr.bottom - r.bottom;
      renderer.setScissorTest(true); renderer.setScissor(x, y, r.width, r.height); renderer.setViewport(x, y, r.width, r.height);
      renderer.render(scene, botCam); renderer.setScissorTest(false);
    }
  }
  window.ankel = { S, sim, setMode, select, ctl, goView };
  setMode('assembled');
  requestAnimationFrame(frame);
})();
