function P = sampleMotionOdometry(P, o1, o2, alpha)
%SAMPLEMOTIONODOMETRY Propagate particles P (Mx3) through the odometry model.
%   alpha = [a1 a2 a3 a4] scale VARIANCES (std = sqrt of the expression):
%     var(rot1)  = a1*rot1^2  + a2*trans^2
%     var(trans) = a3*trans^2 + a4*(rot1^2 + rot2^2)
%     var(rot2)  = a1*rot2^2  + a2*trans^2
[rot1, trans, rot2] = decomposeOdometry(o1, o2);
if trans == 0 && rot2 == 0, return; end
M = size(P, 1);
r1 = rot1  - sqrt(alpha(1)*rot1^2 + alpha(2)*trans^2) * randn(M, 1);
t  = trans - sqrt(alpha(3)*trans^2 + alpha(4)*(rot1^2 + rot2^2)) * randn(M, 1);
r2 = rot2  - sqrt(alpha(1)*rot2^2 + alpha(2)*trans^2) * randn(M, 1);
h = P(:, 3) + r1;
P(:, 1) = P(:, 1) + t .* cos(h);
P(:, 2) = P(:, 2) + t .* sin(h);
P(:, 3) = angleWrap(h + r2);
end
