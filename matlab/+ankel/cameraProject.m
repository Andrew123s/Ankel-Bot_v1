function [bbox, ok, trunc] = cameraProject(lat, fwd, w, hObj, p, roll, dpitch)
%CAMERAPROJECT Pixel bounding box of a floating object (simulation of the camera).
%   lat, fwd  object centre in the body frame [m]
%   w, hObj   object width across the view and height above water [m]
%   roll      vessel roll [rad]; dpitch  vessel pitch added to camera tilt [rad]
%   bbox = [u1 v1 u2 v2] (left, top, right, bottom), clipped to the image the
%   way a detector reports it; ok false if not in view; trunc true if the box
%   was cut by the left, right or bottom image edge (size is then a lower bound).
c = p.cam;  th = c.pitch + dpitch;
Z = fwd - c.fwd;
P = [lat - w/2, c.h,        Z;      % bottom-left  (waterline)
     lat + w/2, c.h,        Z;      % bottom-right
     lat - w/2, c.h - hObj, Z;      % top-left
     lat + w/2, c.h - hObj, Z];     % top-right
uv = zeros(4, 2);  ok = true;  trunc = false;
for i = 1:4
    yc = P(i,2)*cos(th) - P(i,3)*sin(th);
    zc = P(i,2)*sin(th) + P(i,3)*cos(th);
    if zc <= 0.05, ok = false; bbox = [0 0 0 0]; return; end
    a = c.fx * P(i,1) / zc;  b = c.fy * yc / zc;
    uv(i,1) = c.cx + a*cos(roll) - b*sin(roll);     % image-plane roll
    uv(i,2) = c.cy + a*sin(roll) + b*cos(roll);
end
bbox = [min(uv(:,1)), min(uv(:,2)), max(uv(:,1)), max(uv(:,2))];
if bbox(3) <= 0 || bbox(1) >= c.W || bbox(4) <= 0 || bbox(2) >= c.H
    ok = false;  return;
end
trunc = bbox(1) < 0 || bbox(3) > c.W || bbox(4) > c.H;
bbox = [max(bbox(1), 0), max(bbox(2), 0), min(bbox(3), c.W), min(bbox(4), c.H)];
end
