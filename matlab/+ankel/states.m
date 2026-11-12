function S = states()
%STATES Supervisor state codes (identical integers in onboard/ankelbot/supervisor.py).
S.IDLE = 0; S.SEARCH = 1; S.APPROACH = 2; S.AVOID = 3; S.BACKOFF = 4;
S.RETURN = 5; S.DOCKED = 6; S.FAILSAFE = 7;
S.names = {'IDLE','SEARCH','APPROACH','AVOID','BACKOFF','RETURN','DOCKED','FAILSAFE'};
end
