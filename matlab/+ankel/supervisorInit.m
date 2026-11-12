function s = supervisorInit()
%SUPERVISORINIT Initial supervisor memory.
S = ankel.states();
s.state = S.SEARCH;
s.t_state = 0;       % time in current state [s]
s.wp = 1;            % current search waypoint index (1-based)
s.hold_psi = 0;      % heading held during AVOID coast / BACKOFF reverse
s.back_psi = 0;      % heading to turn to at the end of BACKOFF
s.side = -1;         % AVOID pass side: -1 steer to port, +1 steer to starboard
s.t_clear = 0;       % time since the last threat was seen [s]
s.t_block = 0;       % time the mouth has been blocked while the rake is open [s]
s.gate = 0;          % previous rake command
s.t_over = 99;       % time since an oversize object was last tracked inside the funnel [s]
end
