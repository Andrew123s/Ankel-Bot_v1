function [c, tBlock, tOver] = supScan(state, tState, gatePrev, tBlock, tOver, tClear, in, p)
%SUPSCAN Turn this step's inputs into the conditions the mission logic acts on.
%   Shared by ankel.supervisorStep (MATLAB) and the Stateflow chart, so both
%   evaluate identical conditions.
%
%   state, tState   current state code and time in it (already advanced by dt)
%   gatePrev        rake command of the previous step
%   tBlock, tOver   timers (returned updated)
%   tClear          time since the last threat while avoiding
%
%   c.threat, c.thr     nearest object to steer around: [lat fwd width]
%   c.target, c.tgt     nearest collectable object: [lat fwd]
%   c.trigger           back off now
%   c.needDock, c.nearDock
%   c.ok                healthy and battery above critical
%   c.backHold          a back-off manoeuvre is still running
%   c.dockHold          must stay docked
%   c.want              state to be in when none of the above applies
q = p.sup;  S = ankel.states();  dt = p.dt;
corridor = p.hinge_x + p.wing_len*sin(p.wing_nom) + q.corridor_margin;
mg = 0;  if state == S.AVOID, mg = q.hold_margin; end

% An object resting on the CLOSED rake is expected (it waits there until the
% threat has passed). An object blocking the mouth while the rake is OPEN is a jam.
if in.mouth_blocked && gatePrev == 0
    tBlock = tBlock + dt;
else
    tBlock = 0;
end

threat = false;  thr = [0 0 0];  bestThr = inf;
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

if overNear, tOver = 0; else, tOver = min(99, tOver + dt); end
% The mouth sensor cannot tell a waiting bottle from a log. If an oversize object
% was tracked into the funnel moments ago, assume it is the log: never open onto it.
suspect = in.mouth_blocked && tOver < q.t_over;

c.threat = threat;  c.thr = thr;  c.target = target;  c.tgt = tgt;
c.trigger = in.obst_range < q.d_stop || tBlock >= q.t_block || funnel || suspect;
c.needDock = in.net_full || in.soc < q.soc_return;
c.nearDock = hypot(in.x - q.dock(1), in.y - q.dock(2)) < q.dock_radius;
c.ok = in.healthy && in.soc >= q.soc_crit;
c.backHold = state == S.BACKOFF && tState < q.t_back + q.t_turn;
c.dockHold = state == S.DOCKED && (tState < q.t_swap || in.soc < q.soc_return || in.net_full);
if c.needDock
    c.want = S.RETURN;
elseif threat || (state == S.AVOID && tClear < q.t_clear)
    c.want = S.AVOID;
elseif target
    c.want = S.APPROACH;
else
    c.want = S.SEARCH;
end
end
