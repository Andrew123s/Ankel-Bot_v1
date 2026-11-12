classdef World < handle
    %WORLD Everything outside the control software: debris field, camera and
    %   detector model, object tracker, wing/rake actuators, collection physics,
    %   net fill and battery. Shared by ankel.simulate and the Simulink block
    %   ankel.sys.WorldBlock.
    %
    %   Collection physics (body frame):
    %     - inside the wing funnel an object is carried along the wing to the mouth
    %     - at the mouth it is COLLECTED if the rake is open and it fits (<= pass_w),
    %       HELD against the rake if the rake is closed, or JAMMED if the rake is
    %       open and it does not fit (the failure the rejection system must prevent)
    %     - a held item enters when the rake reopens; held/jammed objects are
    %       released when the vessel reverses

    properties
        p; sc
        D                       % debris: pos, w, h, overClass, st, latHold
        Tk                      % tracker memory
        drift = [0 0]           % common surface-drift estimate [m/s]
        areaShift = [0 0]       % integral of drift (moves the search pattern)
        wing; gate = 0
        soc; fill = 0; energy = 0
        t = 0
        jams = 0; holds = 0; swaps = 0
        wErr = zeros(0,3)       % [true width, estimate, range]
        K = 8                   % track rows handed to the supervisor
    end

    methods
        function o = World(sc, p)
            o.p = p;  o.sc = sc;
            rng(sc.seed);
            a = p.sup.area;  nS = sc.n_small;  nB = sc.n_border;  nO = sc.n_over;  N = nS + nB + nO;
            d.pos = [a(1) + 1.5 + rand(N,1)*(a(2) - a(1) - 1.5), a(3) + rand(N,1)*(a(4) - a(3))];
            d.w = [0.06 + 0.22*rand(nS,1); 0.30 + 0.12*rand(nB,1); 0.50 + 0.70*rand(nO,1)];
            d.h = 0.04 + 0.08*rand(N,1);
            d.overClass = false(N,1);
            if nO > 0
                d.h(N-nO+1:N) = 0.10 + 0.15*rand(nO,1);
                d.overClass(N-nO+1:N) = true;      % logs/branches: the detector has a class for them
            end
            d.st = zeros(N,1);                      % 0 free, 1 collected, 2 held, 3 jammed
            d.latHold = zeros(N,1);
            o.D = d;
            tk.n = zeros(N,1);  tk.pos = zeros(N,2);  tk.wbuf = nan(N, p.dec.buf);
            tk.tlast = -inf(N,1);  tk.votes = zeros(N,1);  tk.p0 = zeros(N,2);  tk.t0 = zeros(N,1);
            o.Tk = tk;
            o.wing = [p.wing_nom p.wing_nom];
            o.soc = sc.soc0;
        end

        function w = step(o, x, gateCmd, wingCmd, supState, supTState, P)
            %STEP Advance the world one control period.
            %   x          vessel state [x y psi u v r]
            %   gateCmd    rake command (1 closed), wingCmd [port stbd] rad
            %   supState   supervisor state code, supTState time in that state
            %   P          electrical power drawn over the last period [W]
            %   w.tracks, w.net_full, w.soc, w.obst_range, w.mouth_blocked,
            %   w.healthy, w.area_shift, w.fill, w.wing, w.gate
            p = o.p;  sc = o.sc;  dt = p.dt;  S = ankel.states();
            psi = x(3);  cp = cos(psi);  sp = sin(psi);

            % actuators and battery
            o.wing = o.wing + max(-p.wing_rate*dt, min(p.wing_rate*dt, wingCmd - o.wing));
            o.gate = max(0, min(1, o.gate + max(-dt/p.gate_time, min(dt/p.gate_time, gateCmd - o.gate))));
            o.soc = max(0, o.soc - P*dt/3600/p.E_batt*100);
            o.energy = o.energy + P*dt/3600;

            o.collect(x, cp, sp);

            % net swap at the dock (operator action, modelled as 10 s)
            if supState == S.DOCKED && supTState > 10 && o.fill > 0
                o.fill = 0;  o.swaps = o.swaps + 1;
            end

            w.tracks = o.perceive(x, cp, sp);
            w.net_full = o.fill >= 1;
            w.soc = o.soc;
            w.obst_range = 99;
            w.mouth_blocked = any(o.D.st >= 2);
            w.healthy = o.t < sc.fail_time;
            w.area_shift = o.areaShift;
            w.fill = o.fill;  w.wing = o.wing;  w.gate = o.gate;
            o.t = o.t + dt;
        end
    end

    methods (Access = private)
        function collect(o, x, cp, sp)
            p = o.p;  d = o.D;  N = numel(d.w);  dt = p.dt;
            mouthBusy = any(d.st >= 2);
            for i = 1:N
                if d.st(i) >= 2
                    if x(4) < -0.05
                        d.st(i) = 0;                               % washed off when reversing
                    elseif d.st(i) == 2 && o.gate < 0.5 && ~any(d.st == 3)
                        if d.w(i) <= p.pass_w                      % rake opened: waiting item enters
                            d.st(i) = 1;  o.fill = o.fill + p.pack*0.5*d.w(i)^2*d.h(i)/p.V_net;
                        else
                            d.st(i) = 3;  o.jams = o.jams + 1;     % rake opened onto an oversize object
                        end
                    end
                    if d.st(i) >= 2
                        f = p.hinge_fwd + 0.12;  l = d.latHold(i);
                        d.pos(i,:) = [x(1) + f*cp - l*sp, x(2) + f*sp + l*cp];
                    end
                    continue;
                end
                if d.st(i) ~= 0, continue; end
                d.pos(i,:) = d.pos(i,:) + o.sc.current*dt;
                dx = d.pos(i,1) - x(1);  dy = d.pos(i,2) - x(2);
                fwd = dx*cp + dy*sp;  lat = -dx*sp + dy*cp;
                rr = min(d.w(i)/2, 0.25);
                if lat < 0, aw = o.wing(1); else, aw = o.wing(2); end
                moved = false;
                if fwd > p.hinge_fwd && fwd < p.hinge_fwd + p.wing_len*cos(aw)
                    e = p.hinge_x + (fwd - p.hinge_fwd)*tan(aw);
                    if abs(lat) < e && abs(lat) > e - rr           % riding along the wing
                        lat = sign(lat)*max(e - rr, 0);  moved = true;
                    end
                elseif fwd <= p.hinge_fwd && fwd > p.hinge_fwd - 0.15 && abs(lat) < p.mouth_w/2 + 0.05
                    if o.gate > 0.5 || mouthBusy
                        d.st(i) = 2;  o.holds = o.holds + 1;
                    elseif d.w(i) > p.pass_w
                        d.st(i) = 3;  o.jams = o.jams + 1;
                    else
                        d.st(i) = 1;  o.fill = o.fill + p.pack*0.5*d.w(i)^2*d.h(i)/p.V_net;
                    end
                    d.latHold(i) = max(-0.2, min(0.2, lat));
                    mouthBusy = mouthBusy || d.st(i) >= 2;
                    continue;
                elseif abs(lat) < p.B/2 + rr && fwd <= p.hinge_fwd - 0.15 && fwd > -0.85
                    lat = sign(lat + 1e-9)*(p.B/2 + rr);  moved = true;   % deflected by the hull side
                end
                if moved
                    d.pos(i,:) = [x(1) + fwd*cp - lat*sp, x(2) + fwd*sp + lat*cp];
                end
            end
            o.D = d;
        end

        function tracks = perceive(o, x, cp, sp)
            p = o.p;  sc = o.sc;  d = o.D;  tk = o.Tk;  N = numel(d.w);  t = o.t;  dt = p.dt;
            roll  = sc.noise * 0.03*sin(1.1*t);
            pitch = sc.noise * 0.02*sin(0.8*t + 1);

            % camera + detector + monocular size estimation
            for i = 1:N
                if d.st(i) ~= 0, continue; end
                dx = d.pos(i,1) - x(1);  dy = d.pos(i,2) - x(2);
                fwd = dx*cp + dy*sp;  lat = -dx*sp + dy*cp;
                if fwd < p.cam.fwd + 0.3 || hypot(lat, fwd) > p.det.max_range, continue; end
                [bb, ok, trunc] = ankel.cameraProject(lat, fwd, d.w(i), d.h(i), p, roll, pitch);
                if ~ok || (bb(3) - bb(1)) < p.det.min_px || rand > p.det.p_detect, continue; end
                bb = bb + p.det.sigma_px*sc.noise*randn(1,4);
                bb(4) = min(bb(4), p.cam.H);
                er = p.det.sigma_att*sc.noise*randn(1,2);
                [el, ef, ew, valid] = ankel.estimateFromBBox(bb, p, roll + er(1), pitch + er(2));
                if ~valid || ew > 3 || ef <= 0, continue; end
                wpos = [x(1) + ef*cp - el*sp, x(2) + ef*sp + el*cp];
                if tk.n(i) == 0
                    tk.pos(i,:) = wpos;  tk.p0(i,:) = wpos;  tk.t0(i) = t;
                else
                    tk.pos(i,:) = 0.6*tk.pos(i,:) + 0.4*wpos;
                end
                tk.n(i) = tk.n(i) + 1;  tk.tlast(i) = t;
                wb = tk.wbuf(i,:);  wb = wb(~isnan(wb));
                if ~trunc || isempty(wb) || ew > median(wb)     % a cut-off box only bounds the width from below
                    tk.wbuf(i,:) = [tk.wbuf(i,2:end) ew];
                end
                if d.overClass(i) && rand < p.det.p_class, tk.votes(i) = tk.votes(i) + 1; end
                if ~trunc, o.wErr(end+1,:) = [d.w(i), ew, hypot(lat, fwd)]; end
            end

            % common drift: all floating objects ride the same surface current
            age = tk.tlast - tk.t0;
            use = tk.n > 0 & age > p.dec.drift_min_age & (t - tk.tlast) < 1.0;
            if any(use)
                v = (tk.pos(use,:) - tk.p0(use,:)) ./ age(use);
                vm = median(v, 1);
                spd = hypot(vm(1), vm(2));
                if spd > p.dec.drift_max, vm = vm * p.dec.drift_max / spd; end
                o.drift = (1 - p.dec.drift_alpha)*o.drift + p.dec.drift_alpha*vm;
            end
            live = tk.n > 0;
            tk.pos(live,:) = tk.pos(live,:) + o.drift*dt;      % predict every track forward
            o.areaShift = o.areaShift + o.drift*dt;

            % track list in the body frame (nearest K)
            cand = zeros(N,5);  rngs = inf(N,1);
            for i = 1:N
                if tk.n(i) == 0 || t - tk.tlast(i) > p.dec.t_keep, continue; end
                dx = tk.pos(i,1) - x(1);  dy = tk.pos(i,2) - x(2);
                fwd = dx*cp + dy*sp;  lat = -dx*sp + dy*cp;
                if fwd < -2.5, continue; end
                wb = tk.wbuf(i,:);  wb = wb(~isnan(wb));
                kind = ankel.classifyTrack(tk.wbuf(i,:), tk.n(i), tk.votes(i), p);
                if ~sc.reject_enabled, kind = 1; end
                cand(i,:) = [1 lat fwd median(wb) kind];  rngs(i) = hypot(lat, fwd);
            end
            [~, order] = sort(rngs);
            tracks = zeros(o.K, 5);
            for j = 1:min(o.K, N)
                if isfinite(rngs(order(j))), tracks(j,:) = cand(order(j),:); end
            end
            o.Tk = tk;
        end
    end
end
