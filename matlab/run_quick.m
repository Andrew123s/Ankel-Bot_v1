function run_quick(names)
%RUN_QUICK Print metrics for a list of scenarios (developer smoke run).
addpath(fileparts(mfilename('fullpath')));
if nargin < 1
    names = {'calm','mixed','mixed_no_reject','current','noisy','low_battery','dense','link_loss'};
end
p = ankel.params();
for i = 1:numel(names)
    m = ankel.simulate(ankel.scenario(names{i}), p).metrics;
    fprintf('%-16s collected %2d/%2d (%5.1f%%)  jams %d  holds %2d  backoffs %2d  swaps %d  %5.1f Wh  mae %.3f  %s\n', ...
        names{i}, m.collected, m.n_collectable, m.collected_pct, m.jams, m.holds, m.backoffs, m.swaps, m.energy_Wh, m.width_mae, m.final_state);
end
end
