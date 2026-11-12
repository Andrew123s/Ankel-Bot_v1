function [cmd, st] = controlLaw(st, psi_ref, u_ref, psi, u, r, p)
%CONTROLLAW Heading + speed controller and thrust allocation.
%   st.iu  speed-error integrator
%   cmd    [port stbd] ESC commands in [-1, 1]
%   Yaw has priority: the differential term is allocated first and the common
%   (surge) term takes what thrust is left. The integrator only advances when
%   the surge demand is not saturated (anti-windup).
g = p.gains;
Tf = p.Tmax;  Tr = p.rev * p.Tmax;

e  = ankel.wrapPi(psi_ref - psi);
Mz = g.Kp_psi*e - g.Kd_psi*r;
dT = max(-(Tf+Tr)/2, min((Tf+Tr)/2, Mz/(2*p.b)));

eu = u_ref - u;
iNew = max(-g.iu_max, min(g.iu_max, st.iu + eu*p.dt));
Fx = p.Xu*u_ref + p.Xuu_nom*u_ref*abs(u_ref) + g.Kp_u*eu + g.Ki_u*iNew;

c  = Fx/2;
cs = max(-Tr + abs(dT), min(Tf - abs(dT), c));
if cs == c
    st.iu = iNew;
end
if u_ref == 0
    st.iu = 0;
end
cmd = [ankel.thrustToCmd(cs + dT, p), ankel.thrustToCmd(cs - dT, p)];
end
