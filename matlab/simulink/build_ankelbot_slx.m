function build_ankelbot_slx(scenarioName)
%BUILD_ANKELBOT_SLX Generate the Simulink model AnkelBot.slx.
%
%   Requires Simulink. The four blocks are MATLAB System objects wrapping
%   ankel.BlockCore. run_simulink.m builds the model, simulates it and checks
%   it against ankel.simulate (identical vessel state at every step).
%
%   Signal flow (discrete, 0.1 s):
%       World -> Supervisor -> Controller -> Plant
%         ^          |                         |
%         +--z^-1----+ (rake, wings, state)    |
%         +--z^-1------------------------------+ (x, ug, power)
if nargin < 1, scenarioName = 'mixed'; end
here = fileparts(mfilename('fullpath'));
addpath(fileparts(here));
p = ankel.params();  sc = ankel.scenario(scenarioName);
mdl = 'AnkelBot';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist(fullfile(here, [mdl '.slx']), 'file'), delete(fullfile(here, [mdl '.slx'])); end
new_system(mdl);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num2str(p.dt), 'StopTime', num2str(sc.T));

msys = 'simulink/User-Defined Functions/MATLAB System';
blk = @(n) [mdl '/' n];
add = @(n, cls, pos) add_block(msys, blk(n), 'System', cls, ...
    'SimulateUsing', 'Interpreted execution', 'Position', pos);

add('World',      'ankel.sys.WorldBlock',      [120 100 280 260]);
add('Supervisor', 'ankel.sys.SupervisorBlock', [380 100 540 260]);
add('Controller', 'ankel.sys.ControllerBlock', [640 120 780 220]);
add('Plant',      'ankel.sys.PlantBlock',      [880 120 1020 240]);
set_param(blk('World'), 'Scenario', scenarioName);
set_param(blk('Plant'), 'Current', mat2str(sc.current));
onoff = {'off', 'on'};
set_param(blk('Supervisor'), 'RejectEnabled', onoff{sc.reject_enabled + 1});

% Unit delays that close the two feedback paths (initial conditions = simulate.m)
dly = @(n, ic, pos) add_block('simulink/Discrete/Unit Delay', blk(n), ...
    'InitialCondition', ic, 'SampleTime', num2str(p.dt), 'Position', pos);
dly('z_x',    'zeros(6,1)',                          [880 330 920 360]);
dly('z_ug',   '0',                                   [880 380 920 410]);
dly('z_P',    num2str(p.P_hotel),                    [880 430 920 460]);
dly('z_gate', '0',                                   [380 330 420 360]);
dly('z_wing', mat2str([p.wing_nom p.wing_nom]),      [380 380 420 410]);
dly('z_info', '[1 0]',                               [380 430 420 460]);

L = @(a, b) add_line(mdl, a, b, 'autorouting', 'smart');
% forward chain
L('World/1', 'Supervisor/2');        % tracks
L('World/2', 'Supervisor/3');        % flags
L('World/3', 'Supervisor/4');        % area shift
L('Supervisor/1', 'Controller/1');   % refs
L('Controller/1', 'Plant/1');        % ESC commands
% plant feedback
L('Plant/1', 'z_x/1');   L('Plant/2', 'z_ug/1');   L('Plant/3', 'z_P/1');
L('z_x/1', 'World/1');   L('z_x/1', 'Supervisor/1');   L('z_x/1', 'Controller/2');
L('z_ug/1', 'Controller/3');   L('z_P/1', 'World/5');
% supervisor -> world feedback
L('Supervisor/2', 'z_gate/1');   L('Supervisor/3', 'z_wing/1');   L('Supervisor/4', 'z_info/1');
L('z_gate/1', 'World/2');   L('z_wing/1', 'World/3');   L('z_info/1', 'World/4');

% net fill and wing angles from the world model into the plant drag
add_block('simulink/Signal Routing/Selector', blk('sel_fill'), 'Indices', '1', 'InputPortWidth', '6', 'Position', [640 290 680 320]);
add_block('simulink/Signal Routing/Selector', blk('sel_wing'), 'Indices', '[2 3]', 'InputPortWidth', '6', 'Position', [640 340 680 370]);
L('World/4', 'sel_fill/1');   L('World/4', 'sel_wing/1');
L('sel_fill/1', 'Plant/2');   L('sel_wing/1', 'Plant/3');

% logging
logb = @(n, v, pos) add_block('simulink/Sinks/To Workspace', blk(n), 'VariableName', v, 'SaveFormat', 'Array', 'Position', pos);
logb('log_x', 'sl_x', [1100 130 1180 160]);          L('Plant/1', 'log_x/1');
logb('log_cmd', 'sl_cmd', [880 40 960 70]);          L('Controller/1', 'log_cmd/1');
logb('log_info', 'sl_info', [640 40 720 70]);        L('Supervisor/4', 'log_info/1');
logb('log_status', 'sl_status', [380 40 460 70]);    L('World/4', 'log_status/1');
add_block('simulink/Sinks/Scope', blk('Scope'), 'Position', [1100 200 1140 240]);
L('Plant/1', 'Scope/1');

save_system(mdl, fullfile(here, [mdl '.slx']));
fprintf('Built %s. Run:  out = sim(''%s'');\n', fullfile(here, [mdl '.slx']), mdl);
end
