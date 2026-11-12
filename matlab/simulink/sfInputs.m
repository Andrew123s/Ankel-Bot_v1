function in = sfInputs(x, tracks, flags, areaShift)
%SFINPUTS Pack Simulink signals into the supervisor input structure.
%   flags = [net_full soc obst_range mouth_blocked healthy]
in.x = x(1);  in.y = x(2);  in.psi = x(3);  in.tracks = tracks;
in.area_shift = areaShift(:)';
in.net_full = flags(1) > 0.5;  in.soc = flags(2);  in.obst_range = flags(3);
in.mouth_blocked = flags(4) > 0.5;  in.healthy = flags(5) > 0.5;
end
