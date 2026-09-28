function [est, info] = runParticleFilter(map, recs, prm, showPlot)
%RUNPARTICLEFILTER Monte Carlo Localization over a whole robot log.
%   est  - K x 3 pose estimate (map frame) after every laser scan
%   info - struct with per-scan spread, Neff, and the final particle set
if nargin < 4, showPlot = false; end
isOctave = exist('OCTAVE_VERSION', 'builtin') ~= 0;
M = prm.numParticles;

% ---- 1. initialise: uniform over the start box, rejecting samples in walls
P = zeros(0, 3);
b = prm.startBox;
while size(P, 1) < M
    C = [b(1,1) + diff(b(1,:)) * rand(4*M, 1), ...
         b(2,1) + diff(b(2,:)) * rand(4*M, 1), ...
         b(3,1) + diff(b(3,:)) * rand(4*M, 1)];
    P = [P; C(isFree(map, C(:,1), C(:,2)), :)]; %#ok<AGROW>
end
P = P(1:M, :);
w = ones(M, 1) / M;

% sensor parameters
sp = prm;
if strcmp(prm.sensor, 'beam')
    f = fieldnames(prm.beam);
    for k = 1:numel(f), sp.(f{k}) = prm.beam.(f{k}); end
    sensorFn = @beamModel;
else
    sensorFn = @likelihoodFieldModel;
end

nScans = sum([recs.kind] == 'L');
est = zeros(nScans, 3);
info.spread = zeros(nScans, 1);
info.neff = zeros(nScans, 1);
moved = [0 0];
first = true;
prev = [];
k = 0;

if showPlot
    fig = figure('Name', 'Monte Carlo Localization');
    imagesc([0 map.width], [0 map.height], ~map.occ); colormap(gray); axis xy image; hold on;
    hP = plot(P(:,1), P(:,2), '.', 'Color', [0.85 0.3 0.1], 'MarkerSize', 3);
    hE = plot(nan, nan, 'b-', 'LineWidth', 1.5);
    xlabel('x (cm)'); ylabel('y (cm)');
end

for r = 1:numel(recs)
    rec = recs(r);
    % Drive the motion model with the odometry stamped on the LASER records
    % only: it lags the 'O' stream by one reading, so interleaving the two
    % streams makes the robot jitter back and forth (2x path, fake reversals).
    if rec.kind ~= 'L', continue; end
    k = k + 1;
    % ---- 2. prediction: sample the odometry motion model for EVERY particle
    if ~isempty(prev) && any(rec.odom ~= prev)
        P = sampleMotionOdometry(P, prev, rec.odom, prm.alpha);
        moved = moved + [hypot(rec.odom(1) - prev(1), rec.odom(2) - prev(2)), ...
                         abs(angleWrap(rec.odom(3) - prev(3)))];
    end
    prev = rec.odom;

    % ---- 3. correction: weight by the scan (only after enough motion)
    if first || moved(1) >= prm.minTrans || moved(2) >= prm.minRot
        first = false; moved = [0 0];
        logw = sensorFn(P, rec.ranges, map, sp);
        logw(~isFree(map, P(:,1), P(:,2))) = -Inf;       % inside walls / off map
        if all(isinf(logw)), logw(:) = 0; end
        logw = logw + log(max(w, realmin));
        mx = max(logw);
        w = exp(logw - mx);
        w = w / sum(w);                                   % log-sum-exp normalisation
        info.neff(k) = 1 / sum(w.^2);
        % ---- 4. resample only when the weights have degenerated
        if info.neff(k) < prm.resampleThreshold * M
            idx = lowVarianceResample(w);
            P = P(idx, :);
            w = ones(M, 1) / M;
        end
    else
        info.neff(k) = 1 / sum(w.^2);
    end

    est(k, :) = estimatePose(P, w);
    mu = [sum(w .* P(:,1)), sum(w .* P(:,2))];
    info.spread(k) = sqrt(sum(w .* ((P(:,1) - mu(1)).^2 + (P(:,2) - mu(2)).^2)));

    if showPlot && ishandle(fig)
        set(hP, 'XData', P(:,1), 'YData', P(:,2));
        set(hE, 'XData', est(1:k,1), 'YData', est(1:k,2));
        title(sprintf('scan %d / %d   N_{eff} = %.0f   spread = %.1f cm', k, nScans, ...
              info.neff(k), info.spread(k)));
        if isOctave, drawnow; else, drawnow('limitrate'); end
    end
end
info.particles = P;
info.weights = w;
end
