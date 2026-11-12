function r = headingStep(p, stepRad, uRef, T)
%HEADINGSTEP Closed-loop heading step on the full nonlinear plant.
%   Used to verify the analytic gain design against the real model
%   (thruster lag, quadratic damping, saturation).
if nargin < 4, T = 20; end
n = round(T / p.dt);
x = [0; 0; 0; uRef; 0; 0];  Tact = [0 0];  st.iu = 0;
r.t = (0:n-1)' * p.dt;  r.psi = zeros(n,1);  r.u = zeros(n,1);  r.cmd = zeros(n,2);
for k = 1:n
    [cmd, st] = ankel.controlLaw(st, stepRad, uRef, x(3), x(4), x(6), p);
    [x, Tact] = ankel.plantStep(x, Tact, cmd, p, 0, [p.wing_nom p.wing_nom], [0 0]);
    r.psi(k) = x(3);  r.u(k) = x(4);  r.cmd(k,:) = cmd;
end
r.overshoot_pct = 100 * max(0, max(r.psi) - stepRad) / stepRad;
band = abs(r.psi - stepRad) > 0.05*stepRad;
last = find(band, 1, 'last');
if isempty(last), r.settle_s = 0; else, r.settle_s = r.t(min(last+1, n)); end
i90 = find(r.psi >= 0.9*stepRad, 1);
if isempty(i90), r.rise_s = inf; else, r.rise_s = r.t(i90); end
end
