function v = sfScan(st, ts, gp, tb, to, tClear, x, tracks, flags, areaShift)
%SFSCAN Vector interface between the Stateflow chart and ankel.supScan.
%   v = [threat thrLat thrFwd thrW target tgtLat tgtFwd trigger needDock nearDock
%        ok backHold dockHold want tBlock tOver dt]            (1x17)
%   Called extrinsically from the chart (MISSION entry/during).
persistent p
if isempty(p), p = ankel.params(); end
in = sfInputs(x, tracks, flags, areaShift);
[c, tb, to] = ankel.supScan(st, ts + p.dt, gp, tb, to, tClear, in, p);
v = [c.threat c.thr c.target c.tgt c.trigger c.needDock c.nearDock ...
     c.ok c.backHold c.dockHold c.want tb to p.dt];
end
