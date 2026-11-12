function [x, T, P] = plantStep(x, T, cmd, p, fill, wings, cur)
%PLANTSTEP Advance vessel + thrusters by one control period p.dt.
%   cmd = [port stbd] ESC commands in [-1, 1]. Returns electrical power P [W].
x = x(:);
n = round(p.dt / p.dt_plant);  h = p.dt / n;
Tss = [ankel.cmdToThrust(cmd(1), p), ankel.cmdToThrust(cmd(2), p)];
a = 1 - exp(-h / p.tau_thr);
for k = 1:n
    T = T + (Tss - T) * a;                       % first-order ESC/motor lag
    k1 = ankel.vesselDeriv(x,          T, p, fill, wings, cur);
    k2 = ankel.vesselDeriv(x + h/2*k1, T, p, fill, wings, cur);
    k3 = ankel.vesselDeriv(x + h/2*k2, T, p, fill, wings, cur);
    k4 = ankel.vesselDeriv(x + h*k3,   T, p, fill, wings, cur);
    x = x + h/6 * (k1 + 2*k2 + 2*k3 + k4);
end
x(3) = ankel.wrapPi(x(3));
c = max(-1, min(1, cmd));
P = p.Pmax * (abs(c(1))^3 + abs(c(2))^3) + p.P_hotel;
end
