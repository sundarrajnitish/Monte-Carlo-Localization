function z = raycast(map, x, y, ang, maxRange)
%RAYCAST Expected range along each ray (vectorised sphere tracing).
%   x, y, ang must have the same size. Every ray advances by the distance to
%   the nearest obstacle (minus one cell, at least half a cell), which is
%   exact on a grid and far cheaper than fixed 1-cell stepping.
z = maxRange * ones(size(x));
t = zeros(size(x));
c = cos(ang); s = sin(ang);
active = true(size(x));
half = map.res / 2;
for it = 1:4096
    idx = find(active);
    if isempty(idx), break; end
    px = x(idx) + t(idx) .* c(idx);
    py = y(idx) + t(idx) .* s(idx);
    inside = px >= 0 & py >= 0 & px < map.width & py < map.height;
    ix = min(max(floor(px / map.res) + 1, 1), map.W);
    iy = min(max(floor(py / map.res) + 1, 1), map.H);
    d = map.dist(sub2ind(size(map.dist), iy, ix));
    hit = inside & d == 0;
    z(idx(hit)) = min(t(idx(hit)), maxRange);
    done = hit | ~inside | t(idx) >= maxRange;
    active(idx(done)) = false;
    step = max(d - map.res, half);
    t(idx(~done)) = t(idx(~done)) + step(~done);
end
end
