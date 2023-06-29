function [theta1,theta2] = joint_coordinates(x,y,r1,r2,sign)
% Calculates the joint coordinates for a given position
% Input     Type        Description
% x         1x1 double  x-position
% y         1x1 double  y-position
% r1        1x1 double  upper arm length [m]
% r1        1x1 double  lower arm length [m]
% sign      1x1 double  positive or negative theta2 [-1,1]
% Output    Type        Description
% theta1    1x1 double  angle of first joint
% theta2    1x1 double  relative angle of second joint
c2 = (x^2 + y^2 - r1^2 - r2^2)/(2*r1*r2);
s2 = sqrt(1 - c2^2);
theta2 = sign*atan2(s2,c2); % theta2 is deduced

k1 = r1 + r2*c2;
k2 = r2*s2;
theta1 = atan2(y,x) - sign*atan2(k2,k1); % theta1 is deduced
end