function prm = mclDefaults()
%MCLDEFAULTS Default parameters (identical to the Python reference).
prm.numParticles    = 1000;
prm.alpha           = [0.02 1e-5 0.01 1.0];  % odometry noise (variance scales)
prm.sensor          = 'likelihood';          % 'likelihood' or 'beam'
prm.laserOffset     = 25;                    % cm, laser ahead of robot centre
prm.maxRange        = 8183;                  % cm
prm.beamStride      = 5;                     % use every 5th of 180 beams
prm.temperature     = 1;                     % divide log-likelihood (>1 = softer)
prm.sigmaHit        = 10;                    % cm
% likelihood-field mixture
prm.zHit = 0.9; prm.zRand = 0.1;
% beam-model mixture (used when prm.sensor = 'beam')
prm.beam = struct('zHit', 0.80, 'zShort', 0.05, 'zMax', 0.05, 'zRand', 0.10, ...
                  'lambdaShort', 0.01);
prm.resampleThreshold = 0.5;                 % resample when Neff < 0.5 M
prm.minTrans = 5;                            % cm moved before using a new scan
prm.minRot   = deg2rad(5);
prm.startBox = [1780 1860; 0 56; deg2rad(80) deg2rad(100)];  % x; y; theta
end
