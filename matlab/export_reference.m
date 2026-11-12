function export_reference()
%EXPORT_REFERENCE Hand the MATLAB design to the onboard software.
%   1. config/ankelbot_params.json     every parameter and the designed gains
%   2. onboard/tests/data/matlab_reference.json
%        input/output vectors of each algorithm computed by the MATLAB
%        reference. The Python test-suite replays them, so the code that runs
%        on the vessel is checked against the model it was designed with.
here = fileparts(mfilename('fullpath'));  addpath(here);
root = fileparts(here);
p = ankel.params();
rng(7);

% ---------- 1. parameters ----------
if ~exist(fullfile(root, 'config'), 'dir'), mkdir(fullfile(root, 'config')); end
writeJson(fullfile(root, 'config', 'ankelbot_params.json'), p, true);

% ---------- 2. reference vectors ----------
R = struct();

% thrust map
T = [linspace(-45, 55, 41) 0];
R.thrust.T = T;  R.thrust.cmd = arrayfun(@(v) ankel.thrustToCmd(v, p), T);
c = linspace(-1.1, 1.1, 45);
R.thrust.c = c;  R.thrust.T_of_c = arrayfun(@(v) ankel.cmdToThrust(v, p), c);

% controller
n = 300;  C = zeros(n, 9);
for i = 1:n
    st.iu = 3*rand - 1.5;
    v = [st.iu, 2*pi*rand - pi, round(rand*3)/3*0.75 - 0.3*(rand < 0.2), 2*pi*rand - pi, 1.2*rand - 0.3, 1.2*rand - 0.6];
    if rand < 0.1, v(3) = 0; end
    [cmd, st2] = ankel.controlLaw(st, v(2), v(3), v(4), v(5), v(6), p);
    C(i,:) = [v, cmd, st2.iu];
end
R.control = C;      % [iu psi_ref u_ref psi u r | cmd_port cmd_stbd iu_out]

% camera projection + size estimation
n = 300;  G = zeros(n, 17);
for i = 1:n
    v = [4*rand - 2, 0.95 + 7*rand, 0.05 + 1.1*rand, 0.03 + 0.2*rand, 0.08*rand - 0.04, 0.06*rand - 0.03];
    [bb, ok, tr] = ankel.cameraProject(v(1), v(2), v(3), v(4), p, v(5), v(6));
    if ok
        [el, ef, ew, valid] = ankel.estimateFromBBox(bb, p, v(5), v(6));
    else
        el = 0; ef = 0; ew = 0; valid = false;
    end
    G(i,:) = [v, bb, ok, tr, el, ef, ew, valid, 0];
end
R.camera = G(:,1:16);   % [lat fwd w h roll pitch | u1 v1 u2 v2 ok trunc | lat fwd w valid]

% classifier
cases = {[0.2 0.9 0.2 NaN], 3, 0; [0.5 0.6 0.2], 3, 0; [0.2 0.2 0.2], 3, 2; [0.9 0.9], 2, 2; ...
         [0.34 0.36 0.35 0.351], 4, 0; [NaN NaN NaN], 5, 0; [0.1 0.1 0.1 0.1 0.1], 9, 5};
for i = 1:size(cases, 1)
    R.classify(i).widths = cases{i,1};  R.classify(i).n = cases{i,2};  R.classify(i).votes = cases{i,3};
    R.classify(i).kind = ankel.classifyTrack(cases{i,1}, cases{i,2}, cases{i,3}, p);
end

% plant: open-loop response to a command sequence (validates the Python simulator)
n = 300;  x = [0;0;0;0;0;0];  Tact = [0 0];  Pl = zeros(n, 11);
for k = 1:n
    cmd = [0.6 + 0.4*sin(k/40), 0.5*cos(k/25)];
    wings = [p.wing_nom, p.wing_min + 0.2];
    [x, Tact, Pw] = ankel.plantStep(x, Tact, cmd, p, 0.3, wings, [0.03 -0.02]);
    Pl(k,:) = [cmd, x', Tact, Pw];
end
R.plant.fill = 0.3;  R.plant.wings = wings;  R.plant.current = [0.03 -0.02];
R.plant.trace = Pl;   % [cmd_port cmd_stbd | x y psi u v r | Tport Tstbd | P]

% supervisor: full mission trace covering every state.
% Pick the first seed whose mission exercises avoidance, back-off, return,
% docking and failsafe, so the Python port is checked in every state.
for seed = 1:40
    sc = ankel.scenario('mixed');  sc.seed = seed;  sc.soc0 = 20.2;  sc.fail_time = 290;  sc.T = 300;
    lg = ankel.simulate(sc, p);
    if all(arrayfun(@(c) any(lg.state == c), 1:7)), break; end
end
fprintf('supervisor reference mission: seed %d\n', seed);
n = numel(lg.t);  s = ankel.supervisorInit();
tr = struct('in', cell(1,n), 'out', cell(1,n));
seen = zeros(1, 8);
for k = 1:n
    in = lg.sup_in{k};
    in.x = r6(in.x);  in.y = r6(in.y);  in.psi = r6(in.psi);  in.tracks = r6(in.tracks);
    in.area_shift = r6(in.area_shift);  in.soc = r6(in.soc);
    in.net_full = double(in.net_full);  in.mouth_blocked = double(in.mouth_blocked);  in.healthy = double(in.healthy);
    [s, out] = ankel.supervisorStep(s, in, p);      % recomputed on the rounded inputs
    tr(k).in = in;  tr(k).out = out;  seen(out.state + 1) = seen(out.state + 1) + 1;
end
R.supervisor = tr;
S = ankel.states();
fprintf('supervisor trace: %d steps, time in state [s]:\n', n);
for i = 1:8, fprintf('  %-9s %6.1f\n', S.names{i}, seen(i)*p.dt); end

out = fullfile(root, 'onboard', 'tests', 'data');
if ~exist(out, 'dir'), mkdir(out); end
writeJson(fullfile(out, 'matlab_reference.json'), R, false);
fprintf('wrote config/ankelbot_params.json and onboard/tests/data/matlab_reference.json\n');
end

function v = r6(v)
v = round(v * 1e6) / 1e6;
end

function writeJson(file, data, pretty)
txt = jsonencode(data, 'PrettyPrint', pretty);
fid = fopen(file, 'w');  fwrite(fid, txt);  fclose(fid);
end
