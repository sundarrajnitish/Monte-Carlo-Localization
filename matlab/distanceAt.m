function d = distanceAt(map, x, y)
%DISTANCEAT Distance-field lookup (cm); Inf outside the map.
d = inf(size(x));
in = x >= 0 & y >= 0 & x < map.width & y < map.height;
ix = floor(x(in) / map.res) + 1;
iy = floor(y(in) / map.res) + 1;
d(in) = map.dist(sub2ind(size(map.dist), iy, ix));
end
