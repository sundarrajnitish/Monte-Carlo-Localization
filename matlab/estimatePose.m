function pose = estimatePose(P, w)
%ESTIMATEPOSE Robust pose: weighted mean of the dominant mode.
%   A plain weighted mean of a multi-modal belief lands between the modes,
%   often inside a wall. Histogram the weight on a 50 cm grid, pick the
%   heaviest cell and average only particles within 1 m of it.
key = floor(P(:, 1:2) / 50);
[~, ~, g] = unique(key, 'rows');
mass = accumarray(g, w);
[~, best] = max(mass);
sel = g == best;
c = [sum(w(sel) .* P(sel,1)), sum(w(sel) .* P(sel,2))] / max(sum(w(sel)), realmin);
near = hypot(P(:,1) - c(1), P(:,2) - c(2)) < 100;
ww = w(near) + realmin;
pose = [sum(ww .* P(near,1)) / sum(ww), sum(ww .* P(near,2)) / sum(ww), ...
        atan2(sum(ww .* sin(P(near,3))), sum(ww .* cos(P(near,3))))];
end
