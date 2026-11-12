function add_supervisor_chart(mdl, name, pos)
%ADD_SUPERVISOR_CHART Create the mission supervisor as a Stateflow chart.
%
%   The chart owns the mission structure: which states exist, which
%   transitions connect them and in what priority. The conditions it tests
%   (vector v) and the outputs of each state are computed by the same MATLAB
%   functions the reference ankel.supervisorStep uses (supScan, supEntry,
%   supOutputs), reached through the extrinsic wrappers sfScan/sfEntry/sfOut.
%
%   Hierarchy                         transitions, highest priority first
%   MISSION            scans inputs every step
%     FAILSAFE         <-> ACTIVE on health / critical battery
%     ACTIVE
%       BACKOFF        repeats while the trigger persists, else re-decides
%       DOCKED         holds until swapped and charged
%       NORMAL         -> BACKOFF on trigger, -> DOCKED when at the dock
%         SEARCH, APPROACH, AVOID, RETURN   follow v(14) "want"
%
%   v = [threat thrLat thrFwd thrW target tgtLat tgtFwd trigger needDock
%        nearDock ok backHold dockHold want tBlock tOver dt]
blk = [mdl '/' name];
add_block('sflib/Chart', blk, 'Position', pos);
ch = find(sfroot, '-isa', 'Stateflow.Chart', 'Path', blk);
ch.ActionLanguage = 'MATLAB';
ch.ChartUpdate = 'DISCRETE';
ch.SampleTime = '0.1';

% ---- data (port order = creation order) ----
D = {'x','Input','6'; 'tracks','Input','[8 5]'; 'flags','Input','5'; 'areaShift','Input','2'; ...
     'refs','Output','[1 3]'; 'gate','Output','1'; 'wing','Output','[1 2]'; 'info','Output','[1 2]'; ...
     'v','Local','[1 17]'; 'mem','Local','[1 5]'; 'r','Local','[1 11]'; ...
     'st','Local','1'; 'ts','Local','1'; 'gp','Local','1'; 'first','Local','1'};
for i = 1:size(D, 1)
    d = Stateflow.Data(ch);  d.Name = D{i,1};  d.Scope = D{i,2};
    d.Props.Array.Size = D{i,3};  d.DataType = 'double';
end

ext = 'coder.extrinsic(''sfScan'', ''sfEntry'', ''sfOut'');';
emit = ['r = sfOut(st, ts, mem, v, x, areaShift);' newline ...
        'refs = r(1:3); gate = r(4); wing = r(5:6);' newline ...
        'mem = r(7:11); gp = r(4); info = [st ts];'];
leaf = @(nm, code) sprintf('%s\nentry:\n%s\nst = %d; ts = 0;\nmem = sfEntry(st, mem, v, x);\n%s\nduring:\n%s\nts = ts + v(17);\n%s', ...
    nm, ext, code, emit, ext, emit);

    function s = state(nm, label, p)
        s = Stateflow.State(ch);  s.Position = p;  s.LabelString = label;  s.Name = nm;
    end
    function t = trans(src, dst, label, order, so, do)
        t = Stateflow.Transition(ch);  t.Source = src;  t.Destination = dst;
        t.LabelString = label;
        if nargin >= 5, t.SourceOClock = so;  t.DestinationOClock = do; end
        if ~isempty(order), t.ExecutionOrder = order; end
    end
    function dflt(dst, parent)
        % created inside its parent so it is that state's default transition
        t = Stateflow.Transition(parent);  t.Destination = dst;  t.DestinationOClock = 0;
        p = dst.Position;  xm = p(1) + p(3)/2;
        t.SourceEndPoint = [xm, p(2) - 22];
        t.MidPoint = [xm, p(2) - 11];        % keep the whole arrow inside the parent
    end

% ---- states (containment by geometry) ----
mission = state('MISSION', sprintf(['MISSION\nentry:\n%s\nst = 1; ts = 0; gp = 0; first = 1; mem = [1 0 0 -1 0];\n' ...
    'v = zeros(1, 17);\nv = sfScan(st, ts, gp, 0, 99, mem(5), x, tracks, flags, areaShift);\n' ...
    'during:\n%s\nfirst = 0;\nv = sfScan(st, ts, gp, v(15), v(16), mem(5), x, tracks, flags, areaShift);'], ext, ext), [10 10 1270 900]);
failsafe = state('FAILSAFE', leaf('FAILSAFE', 7), [30 260 270 190]);
active   = state('ACTIVE', 'ACTIVE', [340 200 920 690]);
backoff  = state('BACKOFF', leaf('BACKOFF', 4), [360 280 270 190]);
docked   = state('DOCKED', leaf('DOCKED', 6), [360 530 270 190]);
normal   = state('NORMAL', 'NORMAL', [670 240 570 630]);
search   = state('SEARCH', leaf('SEARCH', 1), [690 310 250 200]);
approach = state('APPROACH', leaf('APPROACH', 2), [970 310 250 200]);
avoid    = state('AVOID', leaf('AVOID', 3), [690 600 250 200]);
ret      = state('RETURN', leaf('RETURN', 5), [970 600 250 200]);
J = Stateflow.Junction(ch);  J.Position.Center = [495 810];  J.Position.Radius = 9;
K = Stateflow.Junction(ch);  K.Position.Center = [495 245];  K.Position.Radius = 9;   % back-off re-entry

% ---- default transitions ----
dflt(mission, ch);  dflt(active, mission);  dflt(normal, active);
% NORMAL's default goes through a junction: on the very first step the chart
% decides like the reference does; afterwards (leaving FAILSAFE) it resumes in SEARCH.
N0 = Stateflow.Junction(ch);  N0.Position.Center = [955 283];  N0.Position.Radius = 8;
t0 = Stateflow.Transition(normal);  t0.Destination = N0;
t0.SourceEndPoint = [955 258];  t0.MidPoint = [955 266];
N = {ret, '[first > 0.5 && v(14) == 5]', 11; avoid, '[first > 0.5 && v(14) == 3]', 1; ...
     approach, '[first > 0.5 && v(14) == 2]', 11; search, '', 1};
for i = 1:4                                       % created inside NORMAL so the default path never leaves it
    tn = Stateflow.Transition(normal);  tn.Source = N0;  tn.Destination = N{i,1};
    tn.DestinationOClock = N{i,3};  tn.LabelString = N{i,2};  tn.ExecutionOrder = i;
end

% ---- transitions ----
trans(active, failsafe, '[v(11) < 0.5]', 1, 9, 3);
trans(failsafe, active, '[v(11) > 0.5]', 1, 2, 10);

trans(backoff, K, '[v(12) < 0.5 && v(8) > 0.5]', 1);   % did not clear: leave and re-enter,
trans(K, backoff, '', 1);                              % which restarts the manoeuvre
trans(backoff, J,       '[v(12) < 0.5]', 2);
trans(docked, backoff,  '[v(13) < 0.5 && v(8) > 0.5]', 1, 0, 6);
trans(docked, J,        '[v(13) < 0.5]', 2, 6, 0);

trans(normal, backoff,  '[v(8) > 0.5]', 1, 9, 3);
trans(normal, docked,   '[v(9) > 0.5 && v(10) > 0.5]', 2, 9, 3);

trans(J, docked,   '[v(9) > 0.5 && v(10) > 0.5]', 1);
trans(J, ret,      '[v(14) == 5]', 2);
trans(J, avoid,    '[v(14) == 3]', 3);
trans(J, approach, '[v(14) == 2]', 4);
trans(J, search,   '', 5);

L = {search, 1; approach, 2; avoid, 3; ret, 5};
G = [0 0; 1 0; 0 1; 1 1];                         % grid position [col row] of each state
CK = {[10.8 4.8], [11.5 6.5], [1.2 7.2];          % {dy+2, dx+2} -> [source, destination] o'clock,
      [8.5 3.5],  [0 0],      [2.5 9.5];          % so arrows run between facing sides inside NORMAL
      [7.8 1.8],  [5.5 0.5],  [4.2 10.2]};
for a = 1:4
    k = 0;
    for b = 1:4
        if a == b, continue; end
        k = k + 1;
        d = G(b,:) - G(a,:);  ck = CK{d(2) + 2, d(1) + 2};
        trans(L{a,1}, L{b,1}, sprintf('[v(14) == %d]', L{b,2}), k, ck(1), ck(2));
    end
end
end
