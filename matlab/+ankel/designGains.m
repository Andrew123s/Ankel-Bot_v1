function g = designGains(p)
%DESIGNGAINS Analytic pole-placement design of the heading and speed loops.
%
%   Heading (PD with rate feedback) on the linearised yaw model
%       m33*psi'' + Nr*psi' = Mz,   Mz = Kp*(psi_ref - psi) - Kd*psi'
%   gives  psi'' + ((Nr+Kd)/m33) psi' + (Kp/m33) psi = (Kp/m33) psi_ref,
%   so for natural frequency wn and damping zeta:
%       Kp = m33*wn^2,   Kd = 2*zeta*wn*m33 - Nr
%   wn is kept well below the thruster lag pole 1/tau so the lag is benign.
%
%   Speed (PI + drag feed-forward) on  m11*u' + d*u = Fx  linearised at the
%   collecting speed u0, d = Xu + 2*Xuu_nom*u0:
%       Kp = 2*zeta*w*m11 - d,   Ki = m11*w^2
m33 = p.Iz + p.Nrd;  m11 = p.m + p.Xud;

g.wn_psi = 1.2;  g.zeta_psi = 0.9;
g.Kp_psi = m33 * g.wn_psi^2;
g.Kd_psi = 2*g.zeta_psi*g.wn_psi*m33 - p.Nr;

u0 = 0.5;  d = p.Xu + 2*p.Xuu_nom*u0;
g.w_u = 0.6;  g.zeta_u = 1.0;
g.Kp_u = 2*g.zeta_u*g.w_u*m11 - d;
g.Ki_u = m11 * g.w_u^2;
g.iu_max = 1.5;                  % integrator clamp [m]
end
