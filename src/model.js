/* Ankel-Bot — parametric engineering model.
   Units: metres. Axes: +Z bow, +Y up, +X port. Waterline at y = 0.
   Every component is built as its own group so it can be exploded,
   assembled step by step, isolated or sectioned. */
(function () {
  const T = THREE;
  const AB = (window.AB = { parts: {}, list: [], mats: {}, props: [], flaps: [], wings: [], shellMats: [], clipMats: [] });
  const root = (AB.root = new T.Group());

  /* ---------- textures ---------- */
  function canvasTex(w, h, draw) {
    const c = document.createElement('canvas');
    c.width = w; c.height = h;
    draw(c.getContext('2d'), w, h);
    const t = new T.CanvasTexture(c);
    t.colorSpace = T.SRGBColorSpace;
    t.anisotropy = 8;
    return t;
  }
  const wireTex = canvasTex(64, 64, (g, w, h) => { g.strokeStyle = '#d4d8dc'; g.lineWidth = 9; g.strokeRect(0, 0, w, h); });
  const netTex = canvasTex(32, 32, (g, w, h) => { g.strokeStyle = '#1f3d2c'; g.lineWidth = 7; g.strokeRect(0, 0, w, h); });
  wireTex.wrapS = wireTex.wrapT = netTex.wrapS = netTex.wrapT = T.RepeatWrapping;

  /* ---------- materials ---------- */
  const std = (o) => new T.MeshStandardMaterial(o);
  const M = (AB.mats = {
    hull: new T.MeshPhysicalMaterial({ color: 0x0c5a68, roughness: 0.34, clearcoat: 0.6, clearcoatRoughness: 0.2 }),
    deck: new T.MeshPhysicalMaterial({ color: 0xf1f3f2, roughness: 0.42, clearcoat: 0.4, clearcoatRoughness: 0.3 }),
    accent: std({ color: 0xb7e325, roughness: 0.5 }),
    enc: std({ color: 0xc6cad0, roughness: 0.55 }),
    alu: std({ color: 0xb4bac1, metalness: 0.9, roughness: 0.38 }),
    aluDark: std({ color: 0x4a4f56, metalness: 0.85, roughness: 0.45 }),
    steel: std({ color: 0xd9dcdf, metalness: 1, roughness: 0.22 }),
    black: std({ color: 0x1a1c1f, metalness: 0.3, roughness: 0.6 }),
    rubber: std({ color: 0x0f1011, roughness: 0.95 }),
    rubberHull: std({ color: 0x0f1011, roughness: 0.95 }),
    white: std({ color: 0xe9ecee, roughness: 0.5 }),
    glass: new T.MeshPhysicalMaterial({ color: 0x0c1014, roughness: 0.05, metalness: 0.3, transparent: true, opacity: 0.78 }),
    pcb: std({ color: 0x1d6b3a, roughness: 0.6 }),
    batt: std({ color: 0x1e4f9a, roughness: 0.5 }),
    blue: std({ color: 0x2b4f8f, roughness: 0.5 }),
    red: std({ color: 0xc8231c, roughness: 0.45 }),
    yellow: std({ color: 0xe8c23a, roughness: 0.7 }),
    brass: std({ color: 0xb08d3c, metalness: 0.9, roughness: 0.35 }),
    cable: std({ color: 0x141414, roughness: 0.8 }),
    cableRed: std({ color: 0xa81e18, roughness: 0.7 }),
    cableBlue: std({ color: 0x1b3f86, roughness: 0.7 }),
    nylon: std({ color: 0x8d9299, roughness: 0.6 }),
  });
  M.duct = M.black.clone(); M.duct.side = T.DoubleSide;
  AB.shellMats.push(M.hull, M.deck, M.enc);
  AB.clipMats.push(M.hull, M.deck, M.enc, M.rubberHull, M.accent);

  /* ---------- helpers ---------- */
  function mk(geo, mat, x, y, z, parent) {
    const m = new T.Mesh(geo, mat);
    m.position.set(x || 0, y || 0, z || 0);
    m.castShadow = m.receiveShadow = true;
    if (parent) parent.add(m);
    return m;
  }
  const box = (w, h, d, mat, x, y, z, p) => mk(new T.BoxGeometry(w, h, d), mat, x, y, z, p);
  function cyl(r, h, mat, x, y, z, p, axis, seg, rTop) {
    const g = new T.CylinderGeometry(rTop === undefined ? r : rTop, r, h, seg || 24);
    if (axis === 'x') g.rotateZ(Math.PI / 2);
    if (axis === 'z') g.rotateX(Math.PI / 2);
    return mk(g, mat, x, y, z, p);
  }
  function tube(pts, r, mat, p) {
    const c = new T.CatmullRomCurve3(pts.map((a) => new T.Vector3(a[0], a[1], a[2])));
    return mk(new T.TubeGeometry(c, pts.length * 12, r, 8), mat, 0, 0, 0, p);
  }
  function wirePanel(w, h, p, x, y, z, rx, ry) {
    const t = wireTex.clone(); t.needsUpdate = true; t.repeat.set(w / 0.025, h / 0.025);
    const m = std({ map: t, alphaTest: 0.5, side: T.DoubleSide, metalness: 0.7, roughness: 0.45 });
    const o = mk(new T.PlaneGeometry(w, h), m, x, y, z, p);
    o.rotation.set(rx || 0, ry || 0, 0); o.castShadow = false;
    return o;
  }
  function bolt(p, x, y, z, axis) {
    cyl(0.011, 0.0016, M.steel, x, y, z, p, axis, 16);
    const o = axis === 'z' ? [0, 0, -0.003] : axis === 'x' ? [0.003, 0, 0] : [0, 0.003, 0];
    cyl(0.007, 0.005, M.steel, x + o[0], y + o[1], z + o[2], p, axis, 6);
  }
  // profile given in (z, y), extruded symmetrically across x
  function extrudeZY(pts, width, bevel) {
    bevel = bevel === undefined ? 0.01 : bevel;
    const s = new T.Shape();
    pts.forEach((q, i) => (i ? s.lineTo(q[0], q[1]) : s.moveTo(q[0], q[1])));
    const d = width - 2 * bevel;
    const g = new T.ExtrudeGeometry(s, { depth: d, bevelEnabled: true, bevelThickness: bevel, bevelSize: bevel, bevelSegments: 2, steps: 1 });
    g.rotateY(-Math.PI / 2); g.translate(d / 2, 0, 0);
    return g;
  }
  const _d = new T.Vector3(), _up = new T.Vector3(0, 1, 0);
  function setStrut(m, a, b) {
    _d.subVectors(b, a); const L = _d.length();
    m.position.copy(a).addScaledVector(_d, 0.5); m.scale.set(1, L, 1);
    m.quaternion.setFromUnitVectors(_up, _d.normalize());
  }
  function part(id, cat, label, explode, anchor, internal) {
    const g = new T.Group(); g.name = id; root.add(g);
    const p = { id, cat, label, group: g, explode: new T.Vector3(explode[0], explode[1], explode[2]), anchor: new T.Vector3(anchor[0], anchor[1], anchor[2]), internal: !!internal, k: 0 };
    AB.parts[id] = p; AB.list.push(p);
    return g;
  }

  /* ================= 1. FLOATING BODY + STRUCTURAL FRAME ================= */
  // Asymmetric lofted hull: flat inner wall (clean tunnel), rounded outer bilge,
  // plan-form tapering to a fine reverse-raked stem, keel rising at bow and stern.
  function hullGeo(sg) {
    const N = 32, sec = [], pos = [];
    for (let i = 0; i <= N; i++) {
      const z = -0.80 + 1.59 * (i / N), f = Math.max(0, (z - 0.30) / 0.49), s = 1 - 0.78 * f * f;
      const keel = z < -0.55 ? -0.15 + 0.08 * Math.pow((-0.55 - z) / 0.25, 2) : z > 0.35 ? -0.15 + 0.10 * Math.pow((z - 0.35) / 0.44, 2) : -0.15;
      const w = -0.12 + 0.24 * s;
      const pts = [[-0.12, 0.28], [w - 0.02 * s, 0.28], [w, 0.26], [w, keel + 0.11], [w - 0.02 * s, keel + 0.03], [w - 0.06 * s, keel], [-0.12 + 0.02 * s, keel], [-0.12, keel + 0.02]];
      sec.push(pts.map((q) => [sg * (0.40 + q[0]), q[1], z - (0.07 * f * f * Math.max(0, q[1])) / 0.28]));
    }
    const tri = (a, b, c) => (sg > 0 ? pos.push(...a, ...b, ...c) : pos.push(...a, ...c, ...b));
    for (let i = 0; i < N; i++) for (let j = 0; j < 8; j++) {
      const a = sec[i][j], b = sec[i][(j + 1) % 8], c = sec[i + 1][(j + 1) % 8], d = sec[i + 1][j];
      tri(a, d, c); tri(a, c, b);
    }
    [0, N].forEach((i) => {
      const c = [0, 0, 0]; sec[i].forEach((q) => { c[0] += q[0] / 8; c[1] += q[1] / 8; c[2] += q[2] / 8; });
      for (let j = 0; j < 8; j++) { const p = sec[i][j], q = sec[i][(j + 1) % 8]; if (i) tri(c, q, p); else tri(c, p, q); }
    });
    const g = new T.BufferGeometry(); g.setAttribute('position', new T.Float32BufferAttribute(pos, 3)); g.computeVertexNormals();
    return g;
  }
  const decalTex = canvasTex(512, 96, (g) => {
    g.fillStyle = '#fff'; g.font = 'bold 62px Arial'; g.textBaseline = 'middle'; g.fillText('ANKEL-BOT', 6, 50);
    g.font = 'bold 22px Arial'; g.fillText('ASV-01', 400, 62);
  });
  [1, -1].forEach((sg) => {
    const s = sg > 0 ? 'P' : 'S';
    const g = part('hull' + s, 1, (sg > 0 ? 'Port' : 'Starboard') + ' hull — wave-piercing foam-filled float', [sg * 0.62, -0.1, 0], [sg * 0.45, 0.1, 0.2]);
    mk(hullGeo(sg), M.hull, 0, 0, 0, g);
    box(0.012, 0.03, 1.05, M.rubberHull, sg * 0.527, 0.19, -0.245, g); // rub rail
    box(0.004, 0.022, 1.05, M.accent, sg * 0.521, 0.04, -0.245, g);    // high-vis boot stripe
    const dm = std({ map: decalTex, transparent: true, roughness: 0.5, polygonOffset: true, polygonOffsetFactor: -2 });
    AB.clipMats.push(dm);
    const d = mk(new T.PlaneGeometry(0.5, 0.094), dm, sg * 0.5215, 0.075, -0.1, g);
    d.rotation.y = (sg * Math.PI) / 2; d.castShadow = false;
  });

  {
    const g = part('frame', 1, 'Bridge frame — 30×30 6061-T6 aluminium extrusion', [0, 0, 0], [0, 0.25, -0.1]);
    [1, -1].forEach((sg) => {
      box(0.03, 0.03, 1.50, M.alu, sg * 0.262, 0.235, -0.03, g); // longitudinal rail bolted to hull inner wall
      [0.62, 0.31, -0.37, -0.775].forEach((z) => box(0.07, 0.004, 0.07, M.alu, sg * 0.235, 0.252, z, g)); // gusset plates
      // stern guide channel for the detachable net frame + anti-foul guard rail
      box(0.034, 0.40, 0.034, M.alu, sg * 0.262, 0.05, -0.83, g);
      box(0.03, 0.02, 0.05, M.alu, sg * 0.272, 0.20, -0.805, g);
      box(0.03, 0.02, 0.05, M.alu, sg * 0.272, -0.06, -0.805, g);
      box(0.03, 0.022, 0.03, M.red, sg * 0.262, 0.262, -0.83, g); // quick-release latch
      tube([[sg * 0.275, -0.02, -0.83], [sg * 0.29, -0.02, -1.0], [sg * 0.29, -0.02, -1.16]], 0.006, M.steel, g);
    });
    [0.62, 0.31, -0.37, -0.775].forEach((z) => box(0.494, 0.03, 0.03, M.alu, 0, 0.235, z, g)); // cross-beams
  }
  {
    const g = part('cowl', 1, 'Foredeck — GRP, carries sensor pylon + solar', [0, 0.5, 0.3], [-0.3, 0.32, 0.5]);
    const sh = new T.Shape(); [[-0.52, 0.31], [0.52, 0.31], [0.52, 0.42], [0.42, 0.74], [-0.42, 0.74], [-0.52, 0.42]].forEach((q, i) => (i ? sh.lineTo(q[0], q[1]) : sh.moveTo(q[0], q[1])));
    const fg = new T.ExtrudeGeometry(sh, { depth: 0.023, bevelEnabled: true, bevelThickness: 0.006, bevelSize: 0.006, bevelSegments: 2 });
    fg.rotateX(Math.PI / 2); fg.translate(0, 0.291, 0);
    mk(fg, M.deck, 0, 0, 0, g);
  }

  /* ================= 2. FRONT WASTE COLLECTION MECHANISM ================= */
  [1, -1].forEach((sg) => {
    const s = sg > 0 ? 'P' : 'S';
    const g = part('wing' + s, 2, 'Guide wing — hinged mesh boom + linear actuator', [sg * 0.55, 0, 0.6], [sg * 0.55, 0.16, 1.1]);
    const hx = sg * 0.30, hz = 0.81;
    cyl(0.012, 0.33, M.steel, hx, 0.055, hz, g); // hinge post
    box(0.07, 0.01, 0.09, M.alu, sg * 0.315, 0.215, 0.775, g);
    box(0.07, 0.01, 0.17, M.alu, sg * 0.315, 0.0, 0.735, g);
    const piv = new T.Group(); piv.position.set(hx, 0, hz); g.add(piv);
    cyl(0.011, 0.80, M.alu, 0, 0.15, 0.41, piv, 'z');
    cyl(0.011, 0.80, M.alu, 0, -0.09, 0.41, piv, 'z');
    [0.03, 0.41, 0.80].forEach((z) => cyl(0.011, 0.24, M.alu, 0, 0.03, z, piv));
    wirePanel(0.78, 0.24, piv, 0, 0.03, 0.41, 0, Math.PI / 2);
    cyl(0.04, 0.13, M.yellow, 0, 0.0, 0.845, piv); // tip float (closed-cell EVA)
    cyl(0.019, 0.04, M.steel, 0, 0.15, 0, piv); cyl(0.019, 0.04, M.steel, 0, -0.09, 0, piv); // hinge knuckles
    box(0.03, 0.03, 0.03, M.alu, 0, 0.165, 0.30, piv); // actuator clevis
    box(0.05, 0.04, 0.04, M.alu, sg * 0.42, 0.165, 0.64, g); // actuator base bracket on hull shoulder
    const body = mk(new T.CylinderGeometry(0.017, 0.017, 1, 16), M.black, 0, 0, 0, g);
    const rod = mk(new T.CylinderGeometry(0.007, 0.007, 1, 12), M.steel, 0, 0, 0, g);
    AB.wings.push({ sg, piv, body, rod, A: new T.Vector3(sg * 0.435, 0.165, 0.64), hx, hz });
  });
  AB.wingAngle = 0.7;
  AB.setWings = function (a) {
    AB.wingAngle = a;
    const B = new T.Vector3(), C = new T.Vector3();
    AB.wings.forEach((w) => {
      w.piv.rotation.y = w.sg * a;
      B.set(w.hx + w.sg * Math.sin(a) * 0.30, 0.165, w.hz + Math.cos(a) * 0.30);
      setStrut(w.rod, w.A, B);
      C.subVectors(B, w.A).setLength(0.17).add(w.A);
      setStrut(w.body, w.A, C);
    });
  };
  AB.setWings(0.7);

  {
    const g = part('gate', 2, 'Non-return flap gate — spring-loaded HDPE flaps', [0, 0.8, 0], [0, 0.1, 0]);
    box(0.5, 0.012, 0.012, M.steel, 0, -0.14, 0, g);
    box(0.5, 0.012, 0.012, M.steel, 0, 0.214, 0, g);
    [1, -1].forEach((sg) => {
      cyl(0.009, 0.36, M.steel, sg * 0.245, 0.04, 0, g);
      const piv = new T.Group(); piv.position.set(sg * 0.245, 0, 0); g.add(piv);
      box(0.005, 0.22, 0.225, M.white, 0, 0.0, -0.118, piv);
      cyl(0.013, 0.03, M.steel, 0, 0.125, 0, piv); cyl(0.013, 0.03, M.steel, 0, -0.125, 0, piv);
      AB.flaps.push({ sg, piv });
    });
  }
  {
    // Oversize reject rake: a tined gate hinged under the foredeck. Stowed flat
    // against the deck it leaves the mouth open; swung down it bars the mouth so
    // nothing wider than the tunnel can enter while the vessel steers around it.
    const g = part('rake', 2, 'Reject rake — servo-driven tined gate at the tunnel mouth', [0, 0.62, 0.45], [0, 0.12, 0.64]);
    const piv = new T.Group(); piv.position.set(0, 0.212, 0.64); g.add(piv);
    cyl(0.008, 0.50, M.steel, 0, 0, 0, piv, 'x', 12);                   // hinge shaft
    for (let i = 0; i < 9; i++) cyl(0.005, 0.30, M.steel, -0.22 + i * 0.055, -0.15, 0, piv, 'y', 8);
    box(0.47, 0.012, 0.012, M.steel, 0, -0.295, 0, piv);                // tine tie-bar
    [1, -1].forEach((sg) => box(0.02, 0.03, 0.03, M.alu, sg * 0.245, 0.212, 0.64, g)); // bearing blocks on the rails
    box(0.06, 0.035, 0.045, M.black, 0.19, 0.2, 0.615, g);              // IP68 servo
    AB.rakePiv = piv;
  }
  AB.setRake = (t) => (AB.rakePiv.rotation.x = (1 - t) * Math.PI / 2);  // t = 1 closed (tines down)
  AB.setRake(0);
  AB.setFlaps = (th) => AB.flaps.forEach((f) => (f.piv.rotation.y = f.sg * th));
  AB.setFlaps(1.1);

  {
    const g = part('hatch', 2, 'Tunnel inspection hatch — powder-coated steel grille', [0, 0.5, 0], [0.2, 0.3, 0.2]);
    [1, -1].forEach((sg) => box(0.035, 0.02, 0.675, M.black, sg * 0.285, 0.29, -0.02, g));
    box(0.60, 0.02, 0.035, M.black, 0, 0.29, 0.30, g); box(0.60, 0.02, 0.035, M.black, 0, 0.29, -0.34, g);
    box(0.55, 0.012, 0.012, M.black, 0, 0.296, 0.08, g); box(0.55, 0.012, 0.012, M.black, 0, 0.296, -0.13, g);
    box(0.012, 0.012, 0.62, M.black, 0.1, 0.296, -0.02, g);
  }

  /* ================= 3. DETACHABLE REAR COLLECTION NET ================= */
  {
    const g = part('net', 3, 'Detachable rear net — slide-in frame + knotless HDPE bag', [0, 0.05, -0.75], [0, 0.1, -1.3]);
    const zf = -0.83;
    [1, -1].forEach((sg) => box(0.012, 0.37, 0.012, M.steel, sg * 0.25, 0.055, zf, g));
    box(0.512, 0.012, 0.012, M.steel, 0, -0.13, zf, g);
    box(0.512, 0.012, 0.012, M.steel, 0, 0.07, zf, g);
    box(0.512, 0.014, 0.014, M.steel, 0, 0.24, zf, g); // lift handle bar
    box(0.14, 0.03, 0.03, M.rubber, 0, 0.24, zf, g);   // grip
    // tapered bag
    const n = 10, z1 = -1.78, sec = [];
    for (let i = 0; i <= n; i++) {
      const t = i / n, w = 0.25 - 0.08 * t;
      sec.push({ z: zf + (z1 - zf) * t, w, top: 0.07 - 0.03 * t - 0.02 * Math.sin(Math.PI * t), bot: -0.13 + 0.02 * t });
    }
    const pos = [], uv = [], tile = 0.012;
    const quad = (a, b, c, d, w, l, l0) => {
      pos.push(...a, ...b, ...c, ...a, ...c, ...d);
      const u = w / tile, v0 = l0 / tile, v1 = (l0 + l) / tile;
      uv.push(0, v0, u, v0, u, v1, 0, v0, u, v1, 0, v1);
    };
    for (let i = 0; i < n; i++) {
      const a = sec[i], b = sec[i + 1], l = Math.abs(b.z - a.z), l0 = i * l;
      quad([-a.w, a.top, a.z], [a.w, a.top, a.z], [b.w, b.top, b.z], [-b.w, b.top, b.z], 2 * a.w, l, l0);
      quad([-a.w, a.bot, a.z], [a.w, a.bot, a.z], [b.w, b.bot, b.z], [-b.w, b.bot, b.z], 2 * a.w, l, l0);
      quad([a.w, a.bot, a.z], [a.w, a.top, a.z], [b.w, b.top, b.z], [b.w, b.bot, b.z], a.top - a.bot, l, l0);
      quad([-a.w, a.bot, a.z], [-a.w, a.top, a.z], [-b.w, b.top, b.z], [-b.w, b.bot, b.z], a.top - a.bot, l, l0);
    }
    const e = sec[n];
    quad([-e.w, e.bot, e.z], [e.w, e.bot, e.z], [e.w, e.top, e.z], [-e.w, e.top, e.z], 2 * e.w, e.top - e.bot, 0);
    const bg = new T.BufferGeometry();
    bg.setAttribute('position', new T.Float32BufferAttribute(pos, 3));
    bg.setAttribute('uv', new T.Float32BufferAttribute(uv, 2));
    bg.computeVertexNormals();
    const bag = mk(bg, std({ map: netTex, alphaTest: 0.5, side: T.DoubleSide, roughness: 0.9 }), 0, 0, 0, g);
    bag.castShadow = false;
    // head-rope floats keep the bag mouth-up and at the surface
    [0.3, 0.62, 0.95].forEach((t) => {
      const s = sec[Math.round(t * n)];
      [1, -1].forEach((sg) => cyl(0.024, 0.08, M.yellow, sg * s.w, s.top + 0.012, s.z, g, 'z', 16));
    });
    [1, -1].forEach((sg) => tube(sec.map((s) => [sg * s.w, s.top + 0.004, s.z]), 0.004, M.white, g)); // head rope
    // cod-end drawstring for emptying
    const ring = mk(new T.TorusGeometry(0.03, 0.005, 8, 20), M.white, 0, -0.03, z1 - 0.012, g);
    ring.rotation.set(0, 0, 0);
    box(0.02, 0.02, 0.03, M.red, 0, -0.03, z1 - 0.03, g);
    AB.netGroup = g;
  }

  /* ================= 4. PROPULSION ================= */
  [1, -1].forEach((sg) => {
    const s = sg > 0 ? 'P' : 'S', x = sg * 0.40;
    const g = part('thr' + s, 4, 'Thruster — 24 V brushless, ducted 3-blade prop', [sg * 0.62, -0.1, -0.5], [sg * 0.4, -0.02, -0.95]);
    box(0.11, 0.13, 0.012, M.alu, x, 0.13, -0.808, g); // transom plate
    cyl(0.014, 0.13, M.steel, x, 0.17, -0.835, g, 'x');  // tilt pivot
    box(0.03, 0.03, 0.06, M.black, x, 0.17, -0.86, g);
    box(0.03, 0.30, 0.05, M.black, x, 0.035, -0.875, g); // strut
    cyl(0.04, 0.18, M.black, x, -0.09, -0.93, g, 'z');   // motor pod
    const nose = mk(new T.ConeGeometry(0.04, 0.06, 24), M.black, x, -0.09, -0.81, g); nose.rotation.x = Math.PI / 2;
    const duct = new T.CylinderGeometry(0.062, 0.057, 0.075, 36, 1, true); duct.rotateX(Math.PI / 2);
    mk(duct, M.duct, x, -0.09, -1.035, g);
    box(0.118, 0.004, 0.03, M.black, x, -0.09, -1.012, g); box(0.004, 0.118, 0.03, M.black, x, -0.09, -1.012, g); // stator vanes
    const pr = new T.Group(); pr.position.set(x, -0.09, -1.045); g.add(pr);
    cyl(0.014, 0.03, M.brass, 0, 0, 0, pr, 'z');
    for (let k = 0; k < 3; k++) {
      const arm = new T.Group(); arm.rotation.z = (k * 2 * Math.PI) / 3; pr.add(arm);
      const b = box(0.024, 0.036, 0.003, M.brass, 0, 0.032, 0, arm); b.rotation.y = 0.6;
    }
    AB.props.push({ sg, obj: pr });
  });

  /* ================= 5. CAMERA + AI DETECTION ================= */
  {
    const g = part('cam', 5, 'AI camera — IP67 dome on raised pylon, 1080p + mmWave radar', [0, 0.95, 0.5], [0, 0.58, 0.58]);
    const py = mk(new T.CylinderGeometry(0.058, 0.085, 0.175, 28), M.deck, 0, 0.384, 0.56, g); py.scale.z = 1.55; // streamlined pylon
    const c = new T.Group(); c.position.set(0, 0.47, 0.575); c.rotation.x = 0.227; g.add(c);
    const sh = new T.Shape(); sh.moveTo(-0.11, -0.11); sh.lineTo(0.11, -0.11); sh.lineTo(0.06, 0.07); sh.lineTo(-0.06, 0.07); sh.closePath();
    const pg = new T.ExtrudeGeometry(sh, { depth: 0.004, bevelEnabled: false }); pg.rotateX(-Math.PI / 2);
    mk(pg, M.alu, 0, 0, 0, c);
    cyl(0.052, 0.03, M.white, 0, 0.019, -0.02, c);
    mk(new T.SphereGeometry(0.043, 28, 14, 0, Math.PI * 2, 0, Math.PI / 2), M.glass, 0, 0.034, -0.02, c);
    mk(new T.SphereGeometry(0.022, 16, 12), M.black, 0, 0.045, -0.02, c);
    cyl(0.011, 0.012, M.rubber, 0, 0.05, 0.0, c, 'z');
    box(0.085, 0.005, 0.04, M.glass, 0, 0.0065, 0.08, c); // radar radome
    [[-0.09, 0.098], [0.09, 0.098], [-0.045, -0.055], [0.045, -0.055]].forEach((q) => bolt(c, q[0], 0.0045, q[1]));
    const an = new T.Object3D(); an.position.set(0, 0.07, 0.05); c.add(an);
    AB.camAnchor = an;
  }
  {
    const g = part('ai', 5, 'Edge AI computer — runs the waste-detection network', [0, 0.72, -0.7], [-0.13, 0.14, -0.5], true);
    box(0.10, 0.003, 0.08, M.pcb, -0.13, 0.102, -0.50, g);
    [[-0.045, -0.035], [0.045, -0.035], [-0.045, 0.035], [0.045, 0.035]].forEach((q) => cyl(0.003, 0.012, M.brass, -0.13 + q[0], 0.095, -0.50 + q[1], g, 'y', 8));
    box(0.07, 0.006, 0.06, M.aluDark, -0.13, 0.108, -0.50, g);
    for (let i = 0; i < 9; i++) box(0.07, 0.02, 0.0025, M.aluDark, -0.13, 0.121, -0.528 + i * 0.007, g);
    cyl(0.02, 0.006, M.black, -0.13, 0.134, -0.50, g);
    box(0.016, 0.012, 0.014, M.steel, -0.175, 0.11, -0.47, g); // RJ45
  }

  /* ================= 6. SENSORS + CONTROL ELECTRONICS ================= */
  {
    const g = part('sensors', 6, 'Sensors — GNSS, telemetry, ultrasonic, fill-level', [0, 0.35, 0], [0.44, 0.68, -0.7]);
    cyl(0.02, 0.008, M.black, 0.44, 0.284, -0.70, g);
    cyl(0.007, 0.38, M.black, 0.44, 0.47, -0.70, g);
    cyl(0.046, 0.018, M.black, 0.44, 0.668, -0.70, g); // GNSS + compass puck
    [-0.72, -0.56].forEach((z) => { cyl(0.011, 0.03, M.black, -0.44, 0.295, z, g); cyl(0.004, 0.16, M.rubber, -0.44, 0.39, z, g, 'y', 8); });
    [1, -1].forEach((sg) => { cyl(0.014, 0.012, M.black, sg * 0.30, 0.279, 0.75, g, 'z'); cyl(0.009, 0.014, M.steel, sg * 0.30, 0.279, 0.751, g, 'z'); });
    box(0.034, 0.012, 0.022, M.black, 0, 0.214, -0.37, g); // ToF net/tunnel level sensor (looks aft-down)
  }
  {
    const g = part('elec', 6, 'Autopilot, 2 × ESC, LTE/RC radio', [0, 0.56, -0.7], [0.1, 0.13, -0.5], true);
    box(0.05, 0.018, 0.085, M.black, 0.0, 0.1, -0.50, g);
    box(0.012, 0.002, 0.03, M.yellow, 0.0, 0.11, -0.50, g);
    [0.11, 0.17].forEach((x) => {
      box(0.045, 0.006, 0.085, M.alu, x, 0.094, -0.50, g);
      for (let i = 0; i < 6; i++) box(0.003, 0.016, 0.085, M.alu, x - 0.02 + i * 0.008, 0.105, -0.50, g);
      box(0.03, 0.001, 0.02, M.blue, x, 0.1135, -0.50, g);
    });
    box(0.075, 0.022, 0.055, M.aluDark, 0.14, 0.102, -0.68, g);
    [0.12, 0.16].forEach((x) => cyl(0.004, 0.014, M.brass, x, 0.102, -0.715, g, 'z', 8));
    tube([[0.0, 0.105, -0.545], [0.03, 0.12, -0.6], [0.1, 0.11, -0.66]], 0.0025, M.cableBlue, g);
    tube([[0.11, 0.1, -0.545], [0.07, 0.115, -0.6], [0.02, 0.105, -0.655]], 0.003, M.cableRed, g);
    tube([[0.17, 0.1, -0.545], [0.1, 0.12, -0.61], [0.03, 0.105, -0.66]], 0.003, M.cableRed, g);
    tube([[0.0, 0.105, -0.46], [0.06, 0.125, -0.44], [0.14, 0.105, -0.455]], 0.002, M.yellow, g);
  }

  /* ================= 7. WATERPROOF ELECTRONICS ENCLOSURE ================= */
  {
    const g = part('encBody', 7, 'IP67 enclosure — gasketed, cable glands', [0, 0.25, -0.7], [-0.2, 0.2, -0.75], true);
    box(0.46, 0.005, 0.35, M.enc, 0, 0.0825, -0.585, g);
    [1, -1].forEach((sg) => box(0.005, 0.135, 0.35, M.enc, sg * 0.2275, 0.1475, -0.585, g));
    box(0.45, 0.135, 0.005, M.enc, 0, 0.1475, -0.7575, g); box(0.45, 0.135, 0.005, M.enc, 0, 0.1475, -0.4125, g);
    box(0.43, 0.003, 0.32, M.alu, 0, 0.0865, -0.585, g); // DIN/back plate
    [1, -1].forEach((sg) => {
      [-0.775, -0.385].forEach((z) => box(0.04, 0.005, 0.045, M.alu, sg * 0.19, 0.2165, z + (z < -0.5 ? 0.012 : -0.012), g)); // hanger tabs to cross-beams
      [-0.50, -0.58, -0.66].forEach((z) => { cyl(0.011, 0.026, M.nylon, sg * 0.24, 0.15, z, g, 'x', 12); cyl(0.014, 0.006, M.nylon, sg * 0.233, 0.15, z, g, 'x', 6); });
      cyl(0.011, 0.026, M.nylon, sg * 0.15, 0.15, -0.402, g, 'z', 12);
    });
    cyl(0.008, 0.01, M.white, 0.2, 0.19, -0.762, g, 'z', 12); // Gore pressure-equalising vent
  }
  {
    const g = part('encLid', 7, 'Enclosure lid — silicone gasket, 6 × M5', [0, 0.92, -0.7], [0, 0.23, -0.585]);
    box(0.455, 0.003, 0.335, M.rubber, 0, 0.2165, -0.585, g);
    box(0.47, 0.006, 0.34, M.enc, 0, 0.221, -0.585, g);
    [-0.215, 0, 0.215].forEach((x) => [-0.735, -0.435].forEach((z) => bolt(g, x, 0.2245, z)));
  }

  /* ================= 8. BATTERY + POWER MANAGEMENT ================= */
  const battTex = canvasTex(256, 128, (g, w, h) => {
    g.fillStyle = '#e8ecf2'; g.fillRect(0, 0, w, h); g.fillStyle = '#16335f'; g.font = 'bold 30px Arial';
    g.fillText('LiFePO4', 14, 42); g.font = 'bold 24px Arial'; g.fillText('25.6 V  40 Ah', 14, 78); g.font = '16px Arial'; g.fillText('8S · BMS 60 A · IP65', 14, 108);
  });
  [1, -1].forEach((sg) => {
    const s = sg > 0 ? 'P' : 'S', x = sg * 0.40;
    const g = part('bat' + s, 8, 'Battery pack — 25.6 V 40 Ah LiFePO4 + BMS', [sg * 0.62, 0.5, 0], [sg * 0.4, 0.03, 0.1], true);
    box(0.15, 0.13, 0.40, M.batt, x, -0.065, -0.02, g);
    box(0.17, 0.006, 0.44, M.alu, x, -0.134, -0.02, g); // tray
    [0.1, -0.14].forEach((z) => box(0.156, 0.134, 0.022, M.rubber, x, -0.064, z, g)); // hold-down straps
    box(0.10, 0.012, 0.11, M.black, x, 0.006, 0.02, g); // BMS
    const l = mk(new T.PlaneGeometry(0.11, 0.055), std({ map: battTex, roughness: 0.6 }), x, 0.0012, -0.075, g); l.rotation.x = -Math.PI / 2; l.castShadow = false;
    cyl(0.009, 0.016, M.red, x - 0.03, 0.008, -0.185, g); cyl(0.009, 0.016, M.black, x + 0.03, 0.008, -0.185, g);
  });
  {
    const g = part('power', 8, 'Power distribution — fuses, DC-DC, isolator, charge port', [0, 0.42, -0.7], [-0.14, 0.13, -0.68], true);
    box(0.075, 0.028, 0.055, M.blue, -0.14, 0.104, -0.68, g); // 24→12/5 V DC-DC
    for (let i = 0; i < 5; i++) box(0.075, 0.004, 0.003, M.alu, -0.14, 0.12, -0.70 + i * 0.01, g);
    box(0.10, 0.02, 0.045, M.black, 0.0, 0.1, -0.68, g);       // fuse block
    [0xd9a514, 0xc8231c, 0x2b6fd0, 0x2f9e4d].forEach((c, i) => box(0.012, 0.01, 0.02, std({ color: c, roughness: 0.4 }), -0.033 + i * 0.022, 0.114, -0.68, g));
    cyl(0.019, 0.014, M.black, -0.10, 0.15, -0.766, g, 'z'); cyl(0.012, 0.016, M.steel, -0.10, 0.15, -0.767, g, 'z'); // charge port
    cyl(0.016, 0.012, M.black, 0.10, 0.15, -0.766, g, 'z'); cyl(0.013, 0.02, M.red, 0.10, 0.15, -0.772, g, 'z');    // main isolator / e-stop
  }

  /* ================= 9. COVERS, HARNESS, BRACKETS, FASTENERS ================= */
  {
    const g = part('harness', 9, 'Wiring harness — marine tinned cable, IP68 connectors', [0, -0.42, 0], [0.26, 0.2, 0.1], true);
    [1, -1].forEach((sg) => {
      tube([[sg * 0.37, 0.02, -0.185], [sg * 0.38, 0.12, -0.3], [sg * 0.33, 0.16, -0.44], [sg * 0.27, 0.15, -0.50], [sg * 0.245, 0.15, -0.50]], 0.006, M.cableRed, g);
      tube([[sg * 0.40, -0.06, -0.902], [sg * 0.40, 0.12, -0.902], [sg * 0.40, 0.21, -0.85], [sg * 0.34, 0.2, -0.73], [sg * 0.27, 0.15, -0.66], [sg * 0.245, 0.15, -0.66]], 0.0055, M.cable, g);
      tube([[sg * 0.42, 0.19, 0.64], [sg * 0.36, 0.23, 0.56], [sg * 0.27, 0.207, 0.42], [sg * 0.27, 0.205, -0.3], [sg * 0.262, 0.2, -0.44], [sg * 0.252, 0.155, -0.56], [sg * 0.245, 0.15, -0.58]], 0.004, M.cable, g);
    });
    tube([[0, 0.46, 0.54], [0, 0.31, 0.5], [0.1, 0.25, 0.45], [0.24, 0.214, 0.36], [0.247, 0.212, -0.2], [0.21, 0.2, -0.36], [0.15, 0.16, -0.385], [0.15, 0.15, -0.4]], 0.004, M.cableBlue, g); // camera GigE
    tube([[0.44, 0.283, -0.70], [0.40, 0.22, -0.69], [0.31, 0.18, -0.64], [0.26, 0.155, -0.60]], 0.003, M.cable, g); // GNSS
  }
  const solarTex = canvasTex(256, 256, (g) => {
    g.fillStyle = '#0b1a33'; g.fillRect(0, 0, 256, 256); g.strokeStyle = '#30486f'; g.lineWidth = 3;
    for (let i = 0; i <= 4; i++) { g.beginPath(); g.moveTo(i * 64, 0); g.lineTo(i * 64, 256); g.moveTo(0, i * 64); g.lineTo(256, i * 64); g.stroke(); }
    g.strokeStyle = '#8f9bb0'; g.lineWidth = 1; for (let i = 0; i < 8; i++) { g.beginPath(); g.moveTo(0, i * 32 + 16); g.lineTo(256, i * 32 + 16); g.stroke(); }
  });
  solarTex.wrapS = solarTex.wrapT = T.RepeatWrapping;
  function solarPanel(p, x, y, z, w, l) {
    const t = solarTex.clone(); t.needsUpdate = true; t.repeat.set(w / 0.21, l / 0.21);
    box(w + 0.012, 0.004, l + 0.012, M.alu, x, y + 0.002, z, p);
    box(w, 0.003, l, new T.MeshPhysicalMaterial({ map: t, roughness: 0.15, clearcoat: 1, clearcoatRoughness: 0.05 }), x, y + 0.0052, z, p).castShadow = false;
  }
  {
    const g = part('solar', 9, 'Solar panels — marine-grade, MPPT trickle charge', [0, 0.85, 0.3], [0.28, 0.32, 0.47]);
    [1, -1].forEach((sg) => solarPanel(g, sg * 0.28, 0.297, 0.475, 0.32, 0.25));
  }
  {
    const g = part('aftDeck', 9, 'Aft deck cover — GRP, seals electronics bay', [0, 0.5, 0], [0.25, 0.3, -0.45]);
    box(0.62, 0.03, 0.46, M.deck, 0, 0.27, -0.57, g);
  }
  {
    const g = part('lids', 9, 'Service lids — battery bays + electronics bay', [0, 0.72, 0], [-0.41, 0.3, 0.1]);
    [1, -1].forEach((sg) => box(0.17, 0.012, 0.52, M.deck, sg * 0.41, 0.288, -0.02, g));
    box(0.42, 0.012, 0.32, M.deck, 0, 0.291, -0.58, g);
    solarPanel(g, 0, 0.297, -0.58, 0.30, 0.20);
  }
  {
    const g = part('fasteners', 9, 'A4 stainless fasteners, lifting handles', [0, 1.05, 0], [-0.53, 0.25, 0.5]);
    [1, -1].forEach((sg) => {
      [-0.07, 0.07].forEach((dx) => [-0.24, 0, 0.24].forEach((dz) => bolt(g, sg * 0.41 + dx, 0.2945, -0.02 + dz)));
      [-0.14, 0, 0.14].forEach((dz) => bolt(g, sg * 0.19, 0.2975, -0.58 + dz));
      [-0.76, -0.40].forEach((z) => bolt(g, sg * 0.295, 0.2855, z));
      bolt(g, sg * 0.47, 0.2985, 0.33); bolt(g, sg * 0.05, 0.2985, 0.33); bolt(g, sg * 0.4, 0.2985, 0.7);
      [-0.25, 0.2].forEach((z) => bolt(g, sg * 0.285, 0.3005, z));
      [0.30, -0.34].forEach((z) => bolt(g, sg * 0.15, 0.3005, z));
      [-0.04, 0.04].forEach((dx) => [-0.045, 0.045].forEach((dy) => bolt(g, sg * 0.40 + dx, 0.13 + dy, -0.8145, 'z')));
      [0.2, -0.5].forEach((z) => {
        const hg = new T.TorusGeometry(0.04, 0.006, 8, 18, Math.PI); hg.rotateX(Math.PI / 2); hg.rotateY((sg * Math.PI) / 2);
        mk(hg, M.steel, sg * 0.528, 0.215, z, g);
      });
    });
  }

  /* ---------- category descriptions (assembly order) ---------- */
  AB.cats = [
    { n: 1, title: 'Floating body & structural frame', step: 'Twin foam-filled hulls are bolted to the aluminium bridge frame; the foredeck ties the bows together.',
      text: 'Catamaran layout: two sealed hulls carry the load and leave an open flow-through tunnel between them. The hulls are asymmetric: flat inner walls give the waste a straight, snag-free channel, while the outer sides taper to fine reverse-raked bows that cut drag and shed weed. Four cross-beams and two rails in 30×30 mm 6061-T6 extrusion tie the hulls together and give every other subsystem a hard mounting point.',
      specs: ['LOA 1.62 m · beam 1.07 m · draft ≈ 0.15 m', 'Hulls: rotomoulded HDPE, closed-cell PU foam fill', 'Asymmetric wave-piercing form, flat tunnel walls', 'Tunnel: 0.52 m wide, open bow to stern', 'Displacement at design waterline ≈ 100 kg'] },
    { n: 2, title: 'Front waste collection mechanism', step: 'Guide wings hinge onto the bow posts; actuators, reject rake, flap gate and inspection hatch are fitted.',
      text: 'Two hinged mesh wings form a funnel that sweeps a 1.6 m swath of surface water into the tunnel as the boat advances. Linear actuators set the sweep angle (narrow for tight channels, wide for open water). A servo-driven reject rake swings down across the mouth when the vision system reports an object too large for the tunnel, so it cannot enter and jam while the boat steers around it. A spring-loaded non-return flap gate lets waste in and stops it washing back out when the boat slows or reverses.',
      specs: ['Wings: 0.80 × 0.24 m, 25 mm welded stainless mesh', 'Sweep 20–60°, IP68 linear actuators, ≈ 105 mm stroke', 'Swath 1.15–1.99 m (1.63 m at 40°)', 'Reject rake: 9 tines at 55 mm pitch, closes in 0.8 s', 'Gate: 2 × HDPE flaps on torsion-spring hinges'] },
    { n: 3, title: 'Detachable rear collection net', step: 'The net frame drops into the stern guide channels and is locked by two quick-release latches.',
      text: 'The team’s key addition. Waste carried through the tunnel is retained in a net bag trailing between the thrusters. The bag is laced to a rigid mouth frame that slides vertically into two stern channels, so a full net is lifted out by its handle and a fresh one dropped in within seconds — no tools, and the boat never leaves the water. Guard rails keep the bag clear of the propeller ducts.',
      specs: ['Mouth 0.50 × 0.20 m, 70 mm above / 130 mm below waterline', 'Bag: 0.95 m knotless HDPE, 10 mm mesh, ≈ 70 L', 'Head-rope floats + cod-end drawstring for emptying', 'Swap: 2 latches, lift handle, no tools'] },
    { n: 4, title: 'Propulsion motors & propellers', step: 'Thruster brackets bolt to each transom; pods sit below the waterline.',
      text: 'Two independently driven ducted thrusters, one behind each hull. Steering is by differential thrust, so there is no rudder to foul. Mounting them on the hulls keeps the propellers outside the waste stream; the ducts and guard rails stop the net and debris reaching the blades.',
      specs: ['2 × 24 V brushless thruster, ≈ 350 W each', '3-blade Ø110 mm prop in Kort-style duct', 'Differential steering, turn-in-place', 'Collecting at 0.5 m/s · model top speed ≈ 1.5 m/s'] },
    { n: 5, title: 'Camera & AI detection system', step: 'The sensor pylon and dome camera mount on the foredeck; the edge computer goes on the enclosure back-plate.',
      text: 'A forward dome camera on a raised bow pylon, 0.55 m above the water for a longer look-ahead, streams video to an embedded GPU computer in the enclosure. A lightweight object-detection network classifies floating items (bottle, bag, can, organic) and returns their bearing; the autopilot converts that bearing into differential thrust so the funnel is steered onto the target.',
      specs: ['IP67 dome, 1080p30, 79° horizontal FOV', 'Edge computer: embedded GPU module, ≈ 15 W', 'Detector: YOLO26n, runs fully offline', 'Box → range + width on the water plane', 'GigE link through IP68 gland'] },
    { n: 6, title: 'Sensors & electronic components', step: 'GNSS mast, antennas, ultrasonic and level sensors are mounted; autopilot, ESCs and radio go in the enclosure.',
      text: 'Navigation and safety sensing around the AI core: GNSS + compass for position hold and return-to-dock, two forward ultrasonic rangers and a mmWave radar for obstacle stand-off, a time-of-flight sensor watching the net mouth for a full bag, and LTE plus 2.4 GHz RC links for telemetry and manual override.',
      specs: ['GNSS/compass puck on 0.4 m mast', 'Autopilot with IMU, 2 × 40 A marine ESC', '2 × waterproof ultrasonic (0.25–4.5 m)', 'LTE telemetry + 2.4 GHz RC failsafe'] },
    { n: 7, title: 'Waterproof electronics enclosure', step: 'The enclosure hangs from the aft cross-beams above the tunnel; the gasketed lid is screwed down.',
      text: 'All control electronics live in one gasketed box hung under the aft deck, 80 mm above the waterline and out of the spray. Every cable enters through a compression gland on the side walls; a membrane vent equalises pressure without admitting water.',
      specs: ['460 × 350 × 135 mm, IP67', 'Silicone gasket, 6 × M5 captive screws', '8 × M16 cable glands, pressure vent', 'Aluminium back-plate as heat spreader'] },
    { n: 8, title: 'Battery & power management', step: 'One battery pack is strapped into each hull; fusing, DC-DC and the isolator are wired in.',
      text: 'Two LiFePO4 packs sit low and amidships, one per hull, which keeps the centre of gravity down and the boat level as the net fills. Each pack has its own BMS; a fused distribution block feeds the ESCs directly at 24 V and a DC-DC converter supplies 12 V / 5 V to the computer and sensors.',
      specs: ['2 × 25.6 V 40 Ah LiFePO4 (≈ 2 kWh total)', 'BMS per pack, 60 A continuous', 'Fused bus + 24→12/5 V DC-DC', 'Stern charge port and main isolator / e-stop', '3 deck solar panels, ≈ 40 Wp via MPPT', '≈ 15 h at 0.5 m/s in the MATLAB model (drag is an estimate)'] },
    { n: 9, title: 'Covers, harness, brackets & fasteners', step: 'Harness is routed and glanded, deck covers and service lids are closed and bolted, handles fitted.',
      text: 'What holds it together and keeps it serviceable: the wiring harness routed under the frame rails, the aft deck cover and three bolted service lids over the battery and electronics bays, deck-mounted solar panels, gusset plates at each frame joint, and four lifting handles for two-person launch.',
      specs: ['A4 (316) stainless M5/M6 throughout', 'Marine tinned cable, IP68 inline connectors', 'GRP covers on neoprene seals', '4 × lifting handles'] },
  ];
})();
