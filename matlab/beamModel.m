function logw = beamModel(P, ranges, map, prm)
%BEAMMODEL Beam (ray-cast) model log-likelihood, Probabilistic Robotics Table 6.1.
%   Mixture of a Gaussian hit around the expected range z*, an exponential
%   'short' reading (unmodelled obstacles), a max-range spike and uniform noise.
idx = 1:prm.beamStride:numel(ranges);
z = ranges(idx);                                       % 1 x B
bearings = deg2rad((idx - 1) - 90);
lx = P(:,1) + prm.laserOffset * cos(P(:,3));
ly = P(:,2) + prm.laserOffset * sin(P(:,3));
ang = P(:,3) + bearings;                               % M x B
zs = raycast(map, repmat(lx, 1, numel(idx)), repmat(ly, 1, numel(idx)), ang, prm.maxRange);
zm = prm.maxRange; s = prm.sigmaHit; lam = prm.lambdaShort;
Z = repmat(z, size(P, 1), 1);
Phi = @(v) 0.5 * (1 + erf(v / sqrt(2)));
etaHit = 1 ./ max(Phi((zm - zs) / s) - Phi(-zs / s), 1e-12);
pHit = etaHit .* exp(-0.5 * ((Z - zs) / s).^2) / (sqrt(2*pi) * s);
pShort = (Z <= zs) .* lam .* exp(-lam * Z) ./ max(1 - exp(-lam * zs), 1e-12);
pMax = double(Z >= zm - 1);
pRand = double(Z < zm) / zm;
p = prm.zHit*pHit + prm.zShort*pShort + prm.zMax*pMax + prm.zRand*pRand;
logw = sum(log(max(p, 1e-300)), 2) / prm.temperature;
end
