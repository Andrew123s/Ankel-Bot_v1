function [s, out] = supervisorStep(s, in, p)
%SUPERVISORSTEP Mission state machine: one decision per control period.
%
%   in.x, in.y, in.psi   pose
%   in.tracks            K-by-5 rows [valid lat fwd width kind], body frame,
%                        kind 0 unknown / 1 collectable / 2 oversize
%   in.area_shift        [dx dy] offset of the search pattern: the pattern rides
%                        with the estimated surface drift so it stays over the waste
%   in.net_full          rear net needs swapping
%   in.soc               battery state of charge [%]
%   in.obst_range        nearest obstacle ahead from ultrasonic/radar [m]
%   in.mouth_blocked     ToF sensor sees an object sitting in the tunnel mouth
%   in.healthy           sensors + link to motor controller OK
%
%   out.state, out.psi_ref, out.u_ref
%   out.gate             1 = reject rake closed across the tunnel mouth
%   out.wing             [port stbd] sweep commands [rad]
%   out.enable           0 = thrusters must be commanded to zero
%
%   This function is the reference implementation; the onboard Python port
%   (onboard/ankelbot/supervisor.py) is tested step-by-step against it.
q = p.sup;  S = ankel.states();  dt = p.dt;
s.t_state = s.t_state + dt;
corridor = p.hinge_x + p.wing_len*sin(p.wing_nom) + q.corridor_margin;
mg = 0;  if s.state == S.AVOID, mg = q.hold_margin; end

% An object resting on the CLOSED rake is expected (it waits there until the
% threat has passed). An object blocking the mouth while the rake is OPEN is a jam.
if in.mouth_blocked && s.gate == 0
    s.t_block = s.t_block + dt;
else
    s.t_block = 0;
end

% ---- scan tracks ----
threat = false;  thr = [0 0 0];  bestThr = inf;      % [lat fwd width]
target = false;  tgt = [0 0];    bestTgt = inf;
funnel = false;  overNear = false;
for k = 1:size(in.tracks, 1)
    tr = in.tracks(k, :);
    if tr(1) < 0.5, continue; end
    lat = tr(2);  fwd = tr(3);  w = tr(4);  kind = tr(5);  rng = hypot(lat, fwd);
    bad = (kind == 2) || (kind == 0 && rng < q.r_gate);
    if bad && fwd > -2.0 && rng < q.r_avoid && abs(lat) < corridor + w/2 + mg && rng < bestThr
        threat = true;  thr = [lat fwd w];  bestThr = rng;
    end
    if kind == 2 && fwd > 0 && fwd < q.funnel_fwd && abs(lat) < q.funnel_lat
        funnel = true;
    end
    if kind == 2 && fwd > 0 && fwd < q.near_fwd && abs(lat) < corridor
        overNear = true;                 % an oversize object is inside the funnel footprint
    end
    if kind == 1 && fwd > 0.3 && rng < bestTgt
        target = true;  tgt = [lat fwd];  bestTgt = rng;
    end
end

% ---- transitions (highest priority first) ----
prev = s.state;  reenter = false;
if overNear, s.t_over = 0; else, s.t_over = min(99, s.t_over + dt); end
% The mouth sensor cannot tell a waiting bottle from a log. If an oversize object
% was tracked into the funnel moments ago, assume it is the log: never open onto it.
suspect = in.mouth_blocked && s.t_over < q.t_over;
trigger = in.obst_range < q.d_stop || s.t_block >= q.t_block || funnel || suspect;
dDock = hypot(in.x - q.dock(1), in.y - q.dock(2));
if ~in.healthy || in.soc < q.soc_crit
    ns = S.FAILSAFE;
elseif prev == S.FAILSAFE
    ns = S.SEARCH;
elseif prev == S.BACKOFF && s.t_state < q.t_back + q.t_turn
    ns = S.BACKOFF;
elseif prev == S.DOCKED && (s.t_state < q.t_swap || in.soc < q.soc_return || in.net_full)
    ns = S.DOCKED;
elseif trigger
    ns = S.BACKOFF;  reenter = prev == S.BACKOFF;   % previous back-off did not clear it: go again
elseif in.net_full || in.soc < q.soc_return
    if dDock < q.dock_radius, ns = S.DOCKED; else, ns = S.RETURN; end
elseif threat
    ns = S.AVOID;
elseif prev == S.AVOID && s.t_clear < q.t_clear
    ns = S.AVOID;
elseif target
    ns = S.APPROACH;
else
    ns = S.SEARCH;
end

% ---- entry actions ----
if ns ~= prev || reenter
    s.t_state = 0;
    if ns == S.BACKOFF
        side = 1;  if threat && thr(1) > 0, side = -1; end
        s.hold_psi = in.psi;
        s.back_psi = ankel.wrapPi(in.psi + side*q.back_turn);
    elseif ns == S.AVOID
        s.side = -1;  if thr(1) < 0, s.side = 1; end
        s.t_clear = 0;
    end
end
s.state = ns;

% ---- outputs ----
out.state = ns;  out.enable = 1;  out.gate = 0;
out.wing = [p.wing_nom p.wing_nom];
out.psi_ref = in.psi;  out.u_ref = 0;
switch ns
    case S.FAILSAFE
        out.enable = 0;  out.gate = 1;
    case S.BACKOFF
        out.gate = 1;
        if s.t_state < q.t_back
            out.u_ref = q.u_back;  out.psi_ref = s.hold_psi;
        else
            out.psi_ref = s.back_psi;
        end
    case S.DOCKED
        out.gate = 1;  out.wing = [p.wing_min p.wing_min];
    case S.RETURN
        out.gate = 1;  out.wing = [p.wing_min p.wing_min];
        out.psi_ref = atan2(q.dock(2) - in.y, q.dock(1) - in.x);
        out.u_ref = q.u_search;
    case S.AVOID
        out.gate = 1;  out.u_ref = q.u_avoid;
        if threat
            rng = max(hypot(thr(1), thr(2)), 0.01);
            c = corridor + thr(3)/2 + q.avoid_extra;
            off = asin(min(1, c/rng));
            s.hold_psi = ankel.wrapPi(in.psi + atan2(thr(1), thr(2)) + s.side*off);
            s.t_clear = 0;
        else
            s.t_clear = s.t_clear + dt;
        end
        out.psi_ref = s.hold_psi;
        if s.side < 0, out.wing(2) = p.wing_min; else, out.wing(1) = p.wing_min; end
    case S.APPROACH
        brg = atan2(tgt(1), tgt(2));
        out.psi_ref = ankel.wrapPi(in.psi + brg);
        if abs(brg) < q.bearing_slow, out.u_ref = q.u_approach; else, out.u_ref = q.u_slow; end
    otherwise  % SEARCH
        wp = q.wps(s.wp, :) + in.area_shift;
        if hypot(wp(1) - in.x, wp(2) - in.y) < q.wp_radius
            s.wp = mod(s.wp, size(q.wps, 1)) + 1;
            wp = q.wps(s.wp, :) + in.area_shift;
        end
        out.psi_ref = atan2(wp(2) - in.y, wp(1) - in.x);
        out.u_ref = q.u_search;
end
s.gate = out.gate;
end
