function T = cmdToThrust(c, p)
%CMDTOTHRUST Steady-state thrust [N] for ESC command c in [-1, 1].
%   Propeller thrust scales with rpm^2 and rpm with command, so T ~ c|c|.
c = max(-1, min(1, c));
if abs(c) < p.cmd_deadband
    T = 0;
elseif c > 0
    T = p.Tmax * c^2;
else
    T = -p.rev * p.Tmax * c^2;
end
end
