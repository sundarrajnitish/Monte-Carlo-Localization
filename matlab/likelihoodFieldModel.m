function logw = likelihoodFieldModel(P, ranges, map, prm)
%LIKELIHOODFIELDMODEL Log-likelihood of a scan for every particle (Table 6.3).
%   Each beam end-point is scored by its distance d to the nearest wall:
%   p = zHit * N(d; 0, sigma) + zRand / maxRange. Max-range beams are skipped.
%   Returns LOG weights - the product of 36+ small densities underflows.
idx = 1:prm.beamStride:numel(ranges);
z = ranges(idx);
keep = z < prm.maxRange - 1;
idx = idx(keep); z = z(keep);
bearings = deg2rad((idx - 1) - 90);                    % 1 x B
lx = P(:,1) + prm.laserOffset * cos(P(:,3));           % M x 1
ly = P(:,2) + prm.laserOffset * sin(P(:,3));
ang = P(:,3) + bearings;                               % M x B (implicit expansion)
ex = lx + z .* cos(ang);
ey = ly + z .* sin(ang);
d = distanceAt(map, ex, ey);
p = prm.zHit * exp(-0.5 * (d / prm.sigmaHit).^2) / (sqrt(2*pi) * prm.sigmaHit) ...
    + prm.zRand / prm.maxRange;
logw = sum(log(p), 2) / prm.temperature;
end
