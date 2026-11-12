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
%   Three implementations of this logic are kept identical by test:
%     - this function (the reference),
%     - the Stateflow chart in matlab/simulink/AnkelBot.slx, which owns the
%       same transitions and calls the same supScan / supEntry / supOutputs,
%     - the onboard Python port (onboard/ankelbot/supervisor.py).
S = ankel.states();
s.t_state = s.t_state + p.dt;
[c, s.t_block, s.t_over] = ankel.supScan(s.state, s.t_state, s.gate, s.t_block, s.t_over, s.t_clear, in, p);

% ---- transitions (highest priority first) ----
prev = s.state;  reenter = false;
if ~c.ok
    ns = S.FAILSAFE;
elseif prev == S.FAILSAFE
    ns = S.SEARCH;
elseif c.backHold
    ns = S.BACKOFF;
elseif c.dockHold
    ns = S.DOCKED;
elseif c.trigger
    ns = S.BACKOFF;  reenter = prev == S.BACKOFF;   % previous back-off did not clear it: go again
elseif c.needDock && c.nearDock
    ns = S.DOCKED;
else
    ns = c.want;
end

if ns ~= prev || reenter
    s.t_state = 0;
    s = ankel.supEntry(ns, s, c, in, p);
end
s.state = ns;

[out, s] = ankel.supOutputs(ns, s.t_state, s, c, in, p);
s.gate = out.gate;
end
