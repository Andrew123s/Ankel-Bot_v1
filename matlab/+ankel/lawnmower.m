function wps = lawnmower(area, lane)
%LAWNMOWER Boustrophedon coverage waypoints over area = [xmin xmax ymin ymax].
%   Lanes run along x, spaced `lane` metres apart in y.
ys = (area(3) + lane/2) : lane : (area(4) - lane/2 + 1e-9);
wps = zeros(2*numel(ys), 2);
for i = 1:numel(ys)
    if mod(i, 2) == 1
        wps(2*i-1, :) = [area(1) ys(i)];  wps(2*i, :) = [area(2) ys(i)];
    else
        wps(2*i-1, :) = [area(2) ys(i)];  wps(2*i, :) = [area(1) ys(i)];
    end
end
end
