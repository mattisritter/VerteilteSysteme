function [c] = parabolic_blend(t0, t1, x0, x1, v0, v1)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
if isnan(x1)
    M = [t0^2, t0, 1;...
         2*t0, 1, 0;...
         2*t1, 1, 0];
    x = [x0; v0; v1];
else
    M = [t1^2, t1, 1;...
         2*t0, 1, 0;...
         2*t1, 1, 0];
    x = [x1; v0; v1];
end
c = M^-1*x;
end

