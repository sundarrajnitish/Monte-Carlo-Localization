function idx = lowVarianceResample(w)
%LOWVARIANCERESAMPLE Systematic resampler (Probabilistic Robotics Table 4.4).
%   w must be normalised. One random offset r ~ U[0, 1/M) and M equally
%   spaced pointers; O(M). If all weights are equal every particle survives.
M = numel(w);
c = cumsum(w(:));
c(end) = 1;                               % guard against round-off
u = rand / M + (0:M-1)' / M;
idx = zeros(M, 1);
i = 1;
for m = 1:M
    while u(m) > c(i)
        i = i + 1;
    end
    idx(m) = i;
end
end
