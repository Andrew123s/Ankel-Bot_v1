function log = simulate(sc, p)
%SIMULATE Closed-loop mission simulation.
%
%   log = ankel.simulate(ankel.scenario('mixed'))
%
%   Loop (one pass per control period p.dt), identical to the Simulink model:
%     World (camera, tracker, collection, battery) -> Supervisor -> Controller -> Plant
%   The same four pieces exist as MATLAB System blocks in +ankel/+sys.
if nargin < 2, p = ankel.params(); end
S = ankel.states();  dt = p.dt;
nSteps = round(sc.T / dt);

world = ankel.World(sc, p);
x = [0; 0; 0; 0; 0; 0];  Tact = [0 0];  P = p.P_hotel;
sup = ankel.supervisorInit();  cst.iu = 0;
gateCmd = 0;  wingCmd = [p.wing_nom p.wing_nom];

log.t = (0:nSteps-1)' * dt;  log.x = zeros(nSteps,6);  log.state = zeros(nSteps,1);
log.cmd = zeros(nSteps,2);  log.gate = zeros(nSteps,1);  log.wing = zeros(nSteps,2);
log.soc = zeros(nSteps,1);  log.fill = zeros(nSteps,1);  log.collected = zeros(nSteps,1);
log.psi_ref = zeros(nSteps,1);  log.u_ref = zeros(nSteps,1);  log.power = zeros(nSteps,1);
log.sup_in = cell(nSteps,1);  log.sup_out = cell(nSteps,1);
backoffs = 0;  dist = 0;

for k = 1:nSteps
    w = world.step(x, gateCmd, wingCmd, sup.state, sup.t_state, P);

    in.x = x(1);  in.y = x(2);  in.psi = x(3);  in.tracks = w.tracks;
    in.area_shift = w.area_shift;  in.net_full = w.net_full;  in.soc = w.soc;
    in.obst_range = w.obst_range;  in.mouth_blocked = w.mouth_blocked;  in.healthy = w.healthy;
    prevState = sup.state;
    [sup, out] = ankel.supervisorStep(sup, in, p);
    if ~sc.reject_enabled, out.gate = 0; end
    if out.state == S.BACKOFF && prevState ~= S.BACKOFF, backoffs = backoffs + 1; end
    gateCmd = out.gate;  wingCmd = out.wing;

    ug = x(4) + sc.current(1)*cos(x(3)) + sc.current(2)*sin(x(3));   % speed over ground along heading
    [cmd, cst] = ankel.controlLaw(cst, out.psi_ref, out.u_ref, x(3), ug, x(6), p);
    if ~out.enable, cmd = [0 0]; end

    xPrev = x;
    [x, Tact, P] = ankel.plantStep(x, Tact, cmd, p, min(w.fill, 1), w.wing, sc.current);
    dist = dist + hypot(x(1) - xPrev(1), x(2) - xPrev(2));

    log.x(k,:) = x';  log.state(k) = out.state;  log.cmd(k,:) = cmd;
    log.gate(k) = w.gate;  log.wing(k,:) = w.wing;  log.soc(k) = w.soc;  log.fill(k) = w.fill;
    log.collected(k) = sum(world.D.st == 1);  log.psi_ref(k) = out.psi_ref;  log.u_ref(k) = out.u_ref;
    log.power(k) = P;  log.sup_in{k} = in;  log.sup_out{k} = out;
end

D = world.D;
fits = D.w <= p.pass_w;
m.n_collectable = sum(fits);
m.collected = sum(D.st == 1);
m.collected_pct = 100 * m.collected / max(1, m.n_collectable);
m.n_policy = sum(D.w <= p.dec.w_max);              % what the size policy is meant to accept
m.collected_policy_pct = 100 * sum(D.st == 1 & D.w <= p.dec.w_max) / max(1, m.n_policy);
m.n_oversize = sum(~fits);
m.jams = world.jams;  m.holds = world.holds;  m.backoffs = backoffs;  m.swaps = world.swaps;
m.energy_Wh = world.energy;  m.distance_m = dist;  m.soc_end = world.soc;
m.drift_est = world.drift;
m.final_state = S.names{sup.state + 1};
if isempty(world.wErr), m.width_mae = NaN; else, m.width_mae = mean(abs(world.wErr(:,2) - world.wErr(:,1))); end
log.metrics = m;  log.debris = D;  log.widthErr = world.wErr;  log.scenario = sc;  log.p = p;
end
