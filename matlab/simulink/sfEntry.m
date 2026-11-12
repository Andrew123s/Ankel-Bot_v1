function mem = sfEntry(st, mem, v, x)
%SFENTRY State entry actions for the Stateflow chart (wraps ankel.supEntry).
%   mem = [wp hold_psi back_psi side t_clear]
persistent p
if isempty(p), p = ankel.params(); end
m = struct('wp', mem(1), 'hold_psi', mem(2), 'back_psi', mem(3), 'side', mem(4), 't_clear', mem(5));
c.threat = v(1) > 0.5;  c.thr = v(2:4);
in.psi = x(3);
m = ankel.supEntry(st, m, c, in, p);
mem = [m.wp m.hold_psi m.back_psi m.side m.t_clear];
end
