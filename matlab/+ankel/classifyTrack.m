function kind = classifyTrack(widths, nObs, overVotes, p)
%CLASSIFYTRACK Decide what to do with a tracked object.
%   widths     recent width estimates [m] (NaN entries ignored)
%   nObs       number of detections so far
%   overVotes  number of detections whose class was an oversize class
%   kind: 0 unknown (too few observations), 1 collectable, 2 oversize
if nObs < p.dec.min_obs
    kind = 0;  return;
end
w = widths(~isnan(widths));
if isempty(w), kind = 0; return; end
if median(w) > p.dec.w_max || overVotes > nObs/2
    kind = 2;
else
    kind = 1;
end
end
