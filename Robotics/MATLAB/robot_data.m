%% MATLAB file for the Robot simulation
% initialize
clear all;

%% User inputs
% define parameters
r1 = 0.21;      % lenght of upper arm [m]
r2 = 0.2452;    % lenght of lower arm [m]
offset = 0.183; % offset of first joint in y [m]

% define timing
t1 = 4;         % time for via point [s]
tf = 10;        % end time [s]

% define obstacle x-position and height
obs = [0.2, 0.1];

% define start point, via and end point
P = [0.10, 0.00 - offset;...
     0.15, 0.15 - offset;...
     0.30, 0.00 - offset];

% define mode of trajectory 
% 'cubic' for two cubic polynomial
% 'quartic' for quartc polynomial
% 'quintic' for two quintic polynomial
% 'hexic' for hexic polynomial
mode = 'quintic';


%% Calculations
ts = 1e-2;          % sample time [s]
t = 0:ts:tf;        % time interval
t01 = 0:ts:t1;      % time interval from start to via time
t1f = ts:ts:tf-t1;  % time interval from via time to end
% calculate joint angles for each point
[theta1_0,theta2_0] = joint_coordinates(P(1,1),P(1,2),r1,r2,-1);
[theta1_1,theta2_1] = joint_coordinates(P(2,1),P(2,2),r1,r2,-1);
[theta1_2,theta2_2] = joint_coordinates(P(3,1),P(3,2),r1,r2,-1); 
Theta = [theta1_0, theta2_0;...
         theta1_1, theta2_1;...
         theta1_2, theta2_2];

% calculating trajectory for each joint
% selects trajectory depending on mode
switch mode
    case 'cubic'
        [c11, c12] = coefficients_cubic_via(Theta(1,1), Theta(2,1), Theta(3,1), t1, tf);
        [c21, c22] = coefficients_cubic_via(Theta(1,2), Theta(2,2), Theta(3,2), t1, tf);
    case 'quartic'
        c11 = coefficients_quartic(Theta(1,1), Theta(2,1), Theta(3,1), t1, tf);
        c12 = NaN(size(c11));
        c21 = coefficients_quartic(Theta(1,2), Theta(2,2), Theta(3,2), t1, tf);
        c22 = NaN(size(c21));
    case 'quintic'
        [c11, c12] = coefficients_quintic_via(Theta(1,1), Theta(2,1), Theta(3,1), t1, tf);
        [c21, c22] = coefficients_quintic_via(Theta(1,2), Theta(2,2), Theta(3,2), t1, tf);
    case 'hexic'
        c11 = coefficients_hexic(Theta(1,1), Theta(2,1), Theta(3,1), t1, tf);
        c12 = NaN(size(c11));
        c21 = coefficients_hexic(Theta(1,2), Theta(2,2), Theta(3,2), t1, tf);
        c22 = NaN(size(c21));
    otherwise
        disp('ERROR: wrong mode')
end

trajectory1 = zeros(size(t));
trajectory2 = zeros(size(t));
if isnan(c12) & isnan(c22)
    trajectory1 = polyval(c11, t);
    trajectory2 = polyval(c21, t);
else
    trajectory1 = [polyval(c11, t01), polyval(c12, t1f)];
    trajectory2 = [polyval(c21, t01), polyval(c22, t1f)];
end

%% Plot Trajectory in cartesian coordinates
% parameters
width = 0.02;      % width of obstacle [m]
w2 = width/2;      % half of the width [m]
% obstacle definition
obstaclex = [obs(1)-w2, obs(1)-w2, obs(1)+w2, obs(1)+w2];
obstacley = [0, obs(2), obs(2), 0];
% precalculation of sine and cosine
sin1 = sin(trajectory1);     % sine of theta1
cos1 = cos(trajectory1);     % cosine of theta1
sin12 = sin(trajectory1+trajectory2);   % sine of theta1+theta2
cos12 = cos(trajectory1+trajectory2);   % cosine of theta1+theta2
% plot
figure(1);
clf;
hold on;
title('Path of the Tool Center Point');
plot(cos1*r1+cos12*r2,sin1*r1+sin12*r2+offset, 'LineWidth', 2);
plot(P(:,1), P(:,2)+offset, 'kx', 'MarkerSize', 10, 'LineWidth', 2);
patch(obstaclex, obstacley, zeros(4,1), 'FaceColor', 'r', 'EdgeColor', 'r');
axis([0 0.4 -0.1 0.3]);
axis manual;
grid on;






