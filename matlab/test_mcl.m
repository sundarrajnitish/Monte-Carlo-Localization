%TEST_MCL Self-checks for the MATLAB implementation (run from this folder).
here = fileparts(mfilename('fullpath'));
map = loadOccupancyMap(fullfile(here, '..', 'data', 'hallway_map.txt'), 4);
assert(isequal(size(map.occ), [85 700]));
assert(isFree(map, 1822, 10) && ~isFree(map, 1500, 10), 'map orientation');

% ray casting in a walled box: inner faces 196 / 96 cm from the centre
box = false(50, 100); box([1 end], :) = true; box(:, [1 end]) = true;
b.occ = box; b.res = 4; [b.H, b.W] = size(box); b.width = 400; b.height = 200;
b.dist = euclideanDistanceTransform(box) * 4;
z = raycast(b, 200 * ones(1, 4), 100 * ones(1, 4), [0 pi/2 pi -pi/2], 8000);
assert(all(abs(z - [196 96 196 96]) <= 4), 'raycast');

% odometry decomposition corner cases
[r1, t, r2] = decomposeOdometry([0 0 0], [10 0 0.5]);   assert(abs(r1) < 1e-12 && abs(t - 10) < 1e-12 && abs(r2 - 0.5) < 1e-12);
[r1, t, r2] = decomposeOdometry([5 5 0.2], [5 5 -0.3]); assert(r1 == 0 && t == 0 && abs(r2 + 0.5) < 1e-12);
[r1, t, ~]  = decomposeOdometry([0 0 0], [-10 0 0]);     assert(abs(t + 10) < 1e-12 && abs(r1) < 1e-12);

% motion noise statistics: std(trans) = sqrt(a3) * trans
P = sampleMotionOdometry(zeros(20000, 3), [0 0 0], [100 0 0], [0 0 0.01 0]);
assert(abs(mean(P(:,1)) - 100) < 0.3 && abs(std(P(:,1)) - 10) < 0.4, 'motion noise');

% low-variance resampling keeps every particle when weights are uniform
assert(isequal(sort(lowVarianceResample(ones(100,1)/100)), (1:100)'));
idx = lowVarianceResample([0 0.5 0.25 0.25]');
assert(~any(idx == 1));

% end to end on the real log
recs = loadRobotLog(fullfile(here, '..', 'data', 'robotdata.log'));
prm = mclDefaults(); prm.numParticles = 300;
est = runParticleFilter(map, recs, prm, false);
assert(all(isFree(map, est(:,1), est(:,2))), 'estimate left free space');
assert(est(end,1) < 800 && est(end,2) > 120 && est(end,2) < 260, 'unexpected final pose');
disp('all MATLAB tests passed');
