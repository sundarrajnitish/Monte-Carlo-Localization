%RUN_MCL Monte Carlo Localization of the robot log in the hallway map.
%   Toolbox-free (base MATLAB R2016b+ or GNU Octave 6+). Run from this folder.
clear; close all; clc;
here = fileparts(mfilename('fullpath'));
dataDir = fullfile(here, '..', 'data');

map  = loadOccupancyMap(fullfile(dataDir, 'hallway_map.txt'), 4);
recs = loadRobotLog(fullfile(dataDir, 'robotdata.log'));
fprintf('map %d x %d cells (%.0f x %.0f cm), %d records, %d scans\n', ...
        map.W, map.H, map.width, map.height, numel(recs), sum([recs.kind] == 'L'));

prm = mclDefaults();
prm.sensor = 'likelihood';        % try 'beam' (slower, slightly more accurate)
rng(0);

tic;
[est, info] = runParticleFilter(map, recs, prm, true);
fprintf('%s model: %.1f ms per scan\n', prm.sensor, 1000 * toc / size(est, 1));
fprintf('final pose: x = %.1f cm, y = %.1f cm, theta = %.1f deg\n', est(end,1), est(end,2), rad2deg(est(end,3)));

figure('Name', 'Filter diagnostics');
subplot(2,1,1); plot(info.spread, 'LineWidth', 1.2); ylabel('spread (cm)'); grid on;
subplot(2,1,2); plot(info.neff, 'LineWidth', 1.2); ylabel('N_{eff}'); xlabel('scan'); grid on;
