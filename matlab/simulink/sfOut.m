function r = sfOut(st, ts, mem, v, x, areaShift)
%SFOUT Outputs of the active state for the Stateflow chart (wraps ankel.supOutputs).
%   r = [psi_ref u_ref enable gate wingPort wingStbd | wp hold_psi back_psi side t_clear]  (1x11)
persistent p
if isempty(p), p = ankel.params(); end
m = struct('wp', mem(1), 'hold_psi', mem(2), 'back_psi', mem(3), 'side', mem(4), 't_clear', mem(5));
c.threat = v(1) > 0.5;  c.thr = v(2:4);  c.tgt = v(6:7);
in.x = x(1);  in.y = x(2);  in.psi = x(3);  in.area_shift = areaShift(:)';
[out, m] = ankel.supOutputs(st, ts, m, c, in, p);
r = [out.psi_ref out.u_ref out.enable out.gate out.wing m.wp m.hold_psi m.back_psi m.side m.t_clear];
end
