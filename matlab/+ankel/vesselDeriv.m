function dx = vesselDeriv(x, T, p, fill, wings, cur)
%VESSELDERIV 3-DOF (surge, sway, yaw) manoeuvring model of the catamaran.
%   x     = [x y psi u v r], u/v are velocities THROUGH THE WATER (body frame)
%   T     = [Tport Tstbd] actual thrust [N]
%   fill  = net fill fraction 0..1 (adds drag)
%   wings = [port stbd] sweep angles [rad] (adds drag)
%   cur   = [cx cy] water current in world frame [m/s]
psi = x(3); u = x(4); v = x(5); r = x(6);
m11 = p.m + p.Xud;  m22 = p.m + p.Yvd;  m33 = p.Iz + p.Nrd;

wingF = (sin(wings(1)) + sin(wings(2))) / (2*sin(p.wing_nom));
Xuu = p.Xuu + p.Xuu_wing*wingF + p.Xuu_net*(1 + p.net_fill_drag*fill);

du = ( m22*v*r - p.Xu*u - Xuu*u*abs(u) + T(1) + T(2)) / m11;
dv = (-m11*u*r - p.Yv*v - p.Yvv*v*abs(v)) / m22;
dr = (-p.Nr*r - p.Nrr*r*abs(r) + p.b*(T(1) - T(2))) / m33;

dx = [u*cos(psi) - v*sin(psi) + cur(1);
      u*sin(psi) + v*cos(psi) + cur(2);
      r; du; dv; dr];
end
