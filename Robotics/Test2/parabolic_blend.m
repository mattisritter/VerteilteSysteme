function [c] = parabolic_blend(t0, t1, x0, v0, v1)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
M = [t0^2, t0, 1;...
     2*t0, 1, 0;...
     2*t1, 1, 0];
x = [x0; v0; v1];
c = M^-1*x;
end

