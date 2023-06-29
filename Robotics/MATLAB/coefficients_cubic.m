function [c] = coefficients_cubic(theta0,thetaf,tf)
% Calculates the coefficients of the cubic polynomial
% Input     Type        Description
% theta0    1x1 double  start angle position
% thetaf    1x1 double  final angle position
% tf        1x1 double  finish time
% Output    Type        Description
% c         4x1 double  coefficients of cubic polynomial
c = zeros(4,1);
M = [0, 0, 0, 1;...
     0, 0, 1, 0;...
     tf^3, tf^2 , tf, 1;...
     3*tf^2, 2*tf, 1, 0];
x = [theta0;...
     0;...
     thetaf;...
     0];
c = M^-1*x;
end

