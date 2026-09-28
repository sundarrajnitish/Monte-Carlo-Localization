function [rot1, trans, rot2] = decomposeOdometry(o1, o2)
%DECOMPOSEODOMETRY Relative odometry motion as (rot1, trans, rot2).
%   Probabilistic Robotics Table 5.6, plus two corner cases:
%   * pure rotation: rot1 = 0, the whole heading change goes into rot2;
%   * reversing: modelled as a negative translation, not a 180 deg spin.
dx = o2(1) - o1(1);
dy = o2(2) - o1(2);
trans = hypot(dx, dy);
dth = angleWrap(o2(3) - o1(3));
if trans < 1e-3
    rot1 = 0; trans = 0; rot2 = dth;
    return
end
rot1 = angleWrap(atan2(dy, dx) - o1(3));
if abs(rot1) > pi/2
    rot1 = angleWrap(rot1 + pi);
    trans = -trans;
end
rot2 = angleWrap(dth - rot1);
end
