function m = supEntry(ns, m, c, in, p)
%SUPENTRY Entry actions of the mission states (run once when a state is entered).
%   m is the supervisor memory: wp, hold_psi, back_psi, side, t_clear.
S = ankel.states();
if ns == S.BACKOFF
    side = 1;  if c.threat && c.thr(1) > 0, side = -1; end       % turn away from the object
    m.hold_psi = in.psi;
    m.back_psi = ankel.wrapPi(in.psi + side*p.sup.back_turn);
elseif ns == S.AVOID
    m.side = -1;  if c.thr(1) < 0, m.side = 1; end               % pass on the far side
    m.t_clear = 0;
end
end
