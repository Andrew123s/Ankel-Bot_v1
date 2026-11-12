function r = run_simulink(scenarioName)
%RUN_SIMULINK Build AnkelBot.slx, simulate it and compare with ankel.simulate.
%   r.max_state_error   largest difference in vessel state over the whole run
%   r.collected, r.jams final counts from the Simulink run and the MATLAB run
%   Saves docs/img/simulink_model.png and docs/img/simulink_vs_matlab.png.
if nargin < 1, scenarioName = 'mixed'; end
here = fileparts(mfilename('fullpath'));  addpath(here, fileparts(here));
root = fileparts(fileparts(here));  img = fullfile(root, 'docs', 'img');
build_ankelbot_slx(scenarioName);
mdl = 'AnkelBot';

out = sim(mdl);
X = squeeze(out.sl_x);  if size(X, 1) == 6 && size(X, 2) ~= 6, X = X'; end
status = squeeze(out.sl_status);  if size(status, 2) ~= 6, status = status'; end

ref = ankel.simulate(ankel.scenario(scenarioName));
n = min(size(X, 1), size(ref.x, 1));
err = abs(X(1:n, :) - ref.x(1:n, :));
r.scenario = scenarioName;
r.steps_compared = n;
r.max_state_error = max(err(:));
r.collected = [status(end, 5), ref.metrics.collected];      % [Simulink, MATLAB]
r.jams = [status(end, 6), ref.metrics.jams];
fprintf('Simulink vs MATLAB (%s): %d steps, max state error %.3g, collected %d / %d, jams %d / %d\n', ...
    scenarioName, n, r.max_state_error, r.collected(1), r.collected(2), r.jams(1), r.jams(2));

if ~exist(img, 'dir'), mkdir(img); end
print(['-s' mdl], '-dpng', '-r110', fullfile(img, 'simulink_model.png'));
f = figure('Visible', 'off', 'Position', [0 0 900 420]);
plot(ref.x(1:n, 2), ref.x(1:n, 1), '-', 'LineWidth', 3, 'Color', [0.75 0.75 0.75]); hold on;
plot(X(1:n, 2), X(1:n, 1), '-', 'LineWidth', 1, 'Color', [0.05 0.35 0.41]);
axis equal; grid on; xlabel('east [m]'); ylabel('north [m]');
legend('ankel.simulate (MATLAB)', 'AnkelBot.slx (Simulink)', 'Location', 'eastoutside');
title(sprintf('Vessel track, scenario "%s": max state difference %.2g', scenarioName, r.max_state_error));
exportgraphics(f, fullfile(img, 'simulink_vs_matlab.png'), 'Resolution', 110); close(f);
close_system(mdl, 0);
end
