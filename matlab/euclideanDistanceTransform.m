function D = euclideanDistanceTransform(occ)
%EUCLIDEANDISTANCETRANSFORM Exact EDT (in cells) to the nearest true cell.
%   Toolbox-free replacement for bwdist, using the separable algorithm of
%   Felzenszwalb & Huttenlocher (2012): 1-D squared transforms along
%   columns, then along rows.
INF = 1e20;
f = INF * ones(size(occ));
f(occ) = 0;
for c = 1:size(f, 2)
    f(:, c) = dt1d(f(:, c));
end
for r = 1:size(f, 1)
    f(r, :) = dt1d(f(r, :).').';
end
D = sqrt(f);
end

function d = dt1d(f)
n = numel(f);
d = zeros(n, 1);
v = zeros(n, 1);          % parabola locations
z = zeros(n + 1, 1);      % boundaries
k = 1; v(1) = 1; z(1) = -Inf; z(2) = Inf;
for q = 2:n
    s = ((f(q) + q^2) - (f(v(k)) + v(k)^2)) / (2*q - 2*v(k));
    while s <= z(k)
        k = k - 1;
        s = ((f(q) + q^2) - (f(v(k)) + v(k)^2)) / (2*q - 2*v(k));
    end
    k = k + 1; v(k) = q; z(k) = s; z(k+1) = Inf;
end
k = 1;
for q = 1:n
    while z(k+1) < q, k = k + 1; end
    d(q) = (q - v(k))^2 + f(v(k));
end
end
