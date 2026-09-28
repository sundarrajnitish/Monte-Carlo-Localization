function a = angleWrap(a)
%ANGLEWRAP Wrap angles to [-pi, pi) (toolbox-free wrapToPi).
a = mod(a + pi, 2*pi) - pi;
end
