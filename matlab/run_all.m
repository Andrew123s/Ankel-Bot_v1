function run_all()
%RUN_ALL Reproduce every MATLAB result of the Ankel-Bot project.
%   1. controller design + verification on the nonlinear plant
%   2. operating scenarios
%   3. Monte-Carlo comparison with and without the rejection system
%   4. requirement tests
%   5. export of parameters, gains and reference vectors to the onboard software
%   Figures go to docs/img, numbers to matlab/results/summary.json.
here = fileparts(mfilename('fullpath'));  addpath(here);
root = fileparts(here);
img = fullfile(root, 'docs', 'img');  res = fullfile(here, 'results');
if ~exist(img, 'dir'), mkdir(img); end
if ~exist(res, 'dir'), mkdir(res); end
p = ankel.params();  S = ankel.states();  R = struct();

%% 1. controller design
r = ankel.headingStep(p, deg2rad(30), 0.5);
R.gains = p.gains;
R.heading_step = struct('step_deg', 30, 'overshoot_pct', r.overshoot_pct, 'settle_s', r.settle_s, 'rise_s', r.rise_s);
fprintf('Heading step 30 deg: overshoot %.1f %%, rise %.1f s, settle %.1f s\n', r.overshoot_pct, r.rise_s, r.settle_s);
f = figure('Visible', 'off', 'Position', [0 0 900 520]);
subplot(2,1,1); plot(r.t, rad2deg(r.psi), 'LineWidth', 1.6); hold on; yline(30, '--'); grid on;
ylabel('heading [deg]'); title('Closed-loop heading step on the nonlinear plant (0.5 m/s)');
subplot(2,1,2); plot(r.t, r.cmd, 'LineWidth', 1.2); grid on; ylabel('ESC command'); xlabel('time [s]'); legend('port', 'starboard');
exportgraphics(f, fullfile(img, 'heading_step.png'), 'Resolution', 110); close(f);

%% 2. scenarios
names = {'calm', 'mixed', 'mixed_no_reject', 'current', 'noisy', 'dense', 'low_battery', 'link_loss'};
for i = 1:numel(names)
    lg = ankel.simulate(ankel.scenario(names{i}), p);  m = lg.metrics;
    R.scenarios.(names{i}) = m;
    fprintf('%-16s collected %2d/%2d (%5.1f%%)  jams %2d  backoffs %2d  swaps %d  %5.1f Wh  %s\n', ...
        names{i}, m.collected, m.n_collectable, m.collected_pct, m.jams, m.backoffs, m.swaps, m.energy_Wh, m.final_state);
    if strcmp(names{i}, 'mixed'), mixed = lg; end
end
R.endurance_h_model = p.E_batt / (R.scenarios.calm.energy_Wh / (ankel.scenario('calm').T / 3600));

% mission map
D = mixed.debris;  a = p.sup.area;
f = figure('Visible', 'off', 'Position', [0 0 900 620]);
plot(mixed.x(:,2), mixed.x(:,1), '-', 'Color', [0.05 0.35 0.41], 'LineWidth', 1); hold on;
rectangle('Position', [a(3) a(1) a(4)-a(3) a(2)-a(1)], 'LineStyle', '--', 'EdgeColor', [0.5 0.5 0.5]);
o = D.w > p.pass_w;  rest = D.st ~= 1 & ~o;
plot(D.pos(rest,2), D.pos(rest,1), 'o', 'Color', [0.2 0.5 0.9], 'MarkerSize', 5);
plot(D.pos(o,2), D.pos(o,1), 'x', 'Color', [0.8 0.15 0.1], 'MarkerSize', 11, 'LineWidth', 2);
plot(p.sup.dock(2), p.sup.dock(1), 's', 'MarkerSize', 12, 'MarkerFaceColor', 'k', 'Color', 'k');
axis equal; grid on; xlabel('east [m]'); ylabel('north [m]');
legend('vessel track', 'search area', 'waste not collected', 'oversize (final position)', 'dock', 'Location', 'eastoutside');
title(sprintf('Scenario "mixed": %d/%d collected, %d jams, %d back-offs', mixed.metrics.collected, mixed.metrics.n_collectable, mixed.metrics.jams, mixed.metrics.backoffs));
exportgraphics(f, fullfile(img, 'mission_map.png'), 'Resolution', 110); close(f);

% timeline
f = figure('Visible', 'off', 'Position', [0 0 900 620]);
subplot(3,1,1); stairs(mixed.t, mixed.state, 'LineWidth', 1); yticks(1:7); yticklabels(S.names(2:8)); ylim([0.5 7.5]); grid on; title('Supervisor state');
subplot(3,1,2); plot(mixed.t, mixed.gate, mixed.t, rad2deg(mixed.wing)/60, 'LineWidth', 1); grid on;
legend('rake (1 = closed)', 'port wing / 60 deg', 'stbd wing / 60 deg', 'Location', 'eastoutside'); title('Rejection actuators');
subplot(3,1,3); plot(mixed.t, mixed.fill, mixed.t, mixed.collected/max(1, mixed.metrics.n_collectable), 'LineWidth', 1.2); grid on;
legend('net fill', 'fraction collected', 'Location', 'eastoutside'); xlabel('time [s]');
exportgraphics(f, fullfile(img, 'timeline.png'), 'Resolution', 110); close(f);

% size estimation
e = mixed.widthErr;
f = figure('Visible', 'off', 'Position', [0 0 760 520]);
scatter(e(:,1), e(:,2), 8, e(:,3), 'filled'); hold on; plot([0 1.3], [0 1.3], 'k--'); xline(p.dec.w_max, 'r:'); yline(p.dec.w_max, 'r:');
cb = colorbar; cb.Label.String = 'range [m]'; grid on; axis([0 1.3 0 1.6]);
xlabel('true width [m]'); ylabel('estimated width [m]'); title(sprintf('Monocular width estimate per detection (MAE %.3f m)', mixed.metrics.width_mae));
exportgraphics(f, fullfile(img, 'size_estimate.png'), 'Resolution', 110); close(f);

%% 3. Monte Carlo: value of the rejection system
nMC = 10;  mc = zeros(nMC, 4);
for sIdx = 1:nMC
    a1 = ankel.scenario('mixed');  a1.seed = sIdx;  m1 = ankel.simulate(a1, p).metrics;
    a2 = ankel.scenario('mixed_no_reject');  a2.seed = sIdx;  m2 = ankel.simulate(a2, p).metrics;
    mc(sIdx,:) = [m1.jams, m1.collected_pct, m2.jams, m2.collected_pct];
end
R.monte_carlo = struct('runs', nMC, 'with_reject_jams_mean', mean(mc(:,1)), 'with_reject_jams_max', max(mc(:,1)), ...
    'with_reject_collected_pct_mean', mean(mc(:,2)), 'no_reject_jams_mean', mean(mc(:,3)), 'no_reject_collected_pct_mean', mean(mc(:,4)));
fprintf('Monte Carlo (%d seeds): jams %.1f (max %d) with rejection vs %.1f without; collected %.1f %% vs %.1f %%\n', ...
    nMC, mean(mc(:,1)), max(mc(:,1)), mean(mc(:,3)), mean(mc(:,2)), mean(mc(:,4)));
f = figure('Visible', 'off', 'Position', [0 0 900 380]);
subplot(1,2,1); bar([mc(:,1) mc(:,3)]); grid on; xlabel('seed'); ylabel('jams in 10 min'); legend('with rejection', 'without', 'Location', 'northwest'); title('Tunnel jams');
subplot(1,2,2); bar([mc(:,2) mc(:,4)]); grid on; xlabel('seed'); ylabel('collectable waste collected [%]'); title('Collection');
exportgraphics(f, fullfile(img, 'rejection_monte_carlo.png'), 'Resolution', 110); close(f);

%% 4. tests
tr = runtests(fullfile(here, 'tests'));
R.tests = struct('passed', sum([tr.Passed]), 'failed', sum([tr.Failed]), 'total', numel(tr));
for i = 1:numel(tr)
    if tr(i).Passed, st = 'PASS'; else, st = 'FAIL'; end
    fprintf('  %s  %s\n', st, tr(i).Name);
end

%% 5. export
export_reference();
fid = fopen(fullfile(res, 'summary.json'), 'w');  fwrite(fid, jsonencode(R, 'PrettyPrint', true));  fclose(fid);
fprintf('Done: %d/%d tests passed. Summary in matlab/results/summary.json\n', R.tests.passed, R.tests.total);
end
