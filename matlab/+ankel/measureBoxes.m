function M = measureBoxes(bb, frameSize, p)
%MEASUREBOXES Detector boxes -> position and width on the water.
%   bb         N-by-4 boxes [x y w h] in frame pixels (MATLAB detector format)
%   frameSize  [rows cols] of the frame the detector saw
%   M          N-by-9: [lat fwd width valid truncated u1 v1 u2 v2]
%              (u, v in calibration pixels, as used by ankel.estimateFromBBox)
%   Used by detectOnVideo and by the Simulink size-estimation block.
c = p.cam;
sx = c.W / frameSize(2);  sy = c.H / frameSize(1);
n = size(bb, 1);  M = zeros(n, 9);
for i = 1:n
    box = [(bb(i,1) - 1)*sx, (bb(i,2) - 1)*sy, (bb(i,1) - 1 + bb(i,3))*sx, (bb(i,2) - 1 + bb(i,4))*sy];
    [lat, fwd, w, valid] = ankel.estimateFromBBox(box, p, 0, 0);
    trunc = box(1) <= 2 || box(3) >= c.W - 2 || box(4) >= c.H - 2;
    M(i,:) = [lat fwd w valid trunc box];
end
end
