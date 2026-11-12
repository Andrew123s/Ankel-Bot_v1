function c = thrustToCmd(T, p)
%THRUSTTOCMD Inverse of cmdToThrust: ESC command that yields thrust T [N].
if T >= 0
    c = sqrt(min(T, p.Tmax) / p.Tmax);
else
    c = -sqrt(min(-T, p.rev*p.Tmax) / (p.rev*p.Tmax));
end
end
