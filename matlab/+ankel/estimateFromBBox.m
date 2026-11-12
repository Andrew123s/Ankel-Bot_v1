function [lat, fwd, w, valid] = estimateFromBBox(bbox, p, roll, dpitch)
%ESTIMATEFROMBBOX Monocular position + width of a floating object.
%   Flat-water assumption: the bottom edge of the box lies on the water plane
%   a known height below the lens, so each bottom corner back-projects to a
%   unique point on the water. Width is the distance between those points.
%   roll / dpitch come from the IMU and remove vessel motion.
%   Outputs are in the body frame; valid is false if a ray misses the water.
c = p.cam;  th = c.pitch + dpitch;
uu = [bbox(1) bbox(3)];  vv = [bbox(4) bbox(4)];
X = zeros(1,2);  Z = zeros(1,2);  valid = true;
for i = 1:2
    a0 = uu(i) - c.cx;  b0 = vv(i) - c.cy;
    a =  a0*cos(roll) + b0*sin(roll);              % undo image-plane roll
    b = -a0*sin(roll) + b0*cos(roll);
    dx = a / c.fx;  dy = b / c.fy;
    Yd = dy*cos(th) + sin(th);                     % downward component
    Zf = -dy*sin(th) + cos(th);                    % forward component
    if Yd <= 1e-6, valid = false; lat = 0; fwd = 0; w = 0; return; end
    t = c.h / Yd;
    X(i) = t*dx;  Z(i) = t*Zf;
end
w   = hypot(X(2) - X(1), Z(2) - Z(1));
lat = (X(1) + X(2)) / 2;
fwd = (Z(1) + Z(2)) / 2 + c.fwd;
end
