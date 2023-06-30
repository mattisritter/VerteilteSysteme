%% MATLAB file for trajectory plannning
clear all;
%% without obstacles
% parameters
theta0 = 15; % starting joint angle [°]
thetaf = 65; % final joint angle [°]

tf = 10; % finish time [s]

ts = 1e-2; % sampling time [s]
t = 0:ts:tf; % time interval

% polynomial coefficients
% calculating coefficients of cubic polynomial
c3 = coefficients_cubic(theta0, thetaf, tf);
% calculating coefficients of quintic polynomial
c5 = coefficients_quintic(theta0, thetaf, tf);

% polynomial functions
% evaluating trajectory, velocity and acceleration of cubic polynomial
trajectory3 = polyval(c3, t);
c3d = polyder(c3);
velocity3 = polyval(c3d, t);
c3dd = polyder(c3d);
acceleration3 = polyval(c3dd, t);
% evaluating trajectory, velocity and acceleration of quintic polynomial
trajectory5 = polyval(c5, t);
c5d = polyder(c5);
velocity5 = polyval(c5d, t);
c5dd = polyder(c5d);
acceleration5 = polyval(c5dd, t);

% plots
% plotting angle acceleration for cubic and quintic trajectory
figure(3);
clf;
plot(t, acceleration3, t, acceleration5); grid on;
title('Angle acceleration for cubic and quintic trajectory');
xlabel('$t$ in $s$','Interpreter','latex');
ylabel('$\ddot{\theta}$ in $^{\circ}/s^2$','Interpreter','latex');
legend('cubic', 'quintic');

% plotting angle velocity for cubic and quintic trajectory
figure(2);
clf;
plot(t, velocity3, t, velocity5); grid on;
title('Angle velocity for cubic and quintic trajectory');
xlabel('$t$ in $s$','Interpreter','latex');
ylabel('$\dot{\theta}$ in $^{\circ}/s$','Interpreter','latex');
legend('cubic', 'quintic');

% plotting angle position for cubic and quintic trajectory
figure(1);
clf;
plot(t, trajectory3, t, trajectory5); grid on;
title('Angle position for cubic and quintic trajectory');
xlabel('$t$ in $s$','Interpreter','latex');
ylabel('$\theta$ in $^{\circ}$','Interpreter','latex');
legend('cubic', 'quintic');

%% with obstacle
% parameters
theta1 = 5; % via joint angle [°]

t1 = 2; % via time [s]

t01 = 0:ts:t1; % time interval from start to via time
t1f = ts:ts:tf-t1; % time interval from via time to end

% polynomial coefficients
% calculating coefficients of quartic polynomial
c4 = coefficients_quartic(theta0, theta1, thetaf, t1, tf);
% calculating coefficients of hexic polynomial
c6 = coefficients_hexic(theta0, theta1, thetaf, t1, tf);
% calculating coefficients of two cubic polynomial
[c31, c32] = coefficients_cubic_via(theta0, theta1, thetaf, t1, tf);
% calculating coefficients of two quintic polynomial
[c51, c52] = coefficients_quintic_via(theta0, theta1, thetaf, t1, tf);

% polynomial functions
% evaluating trajectory, velocity and acceleration of quartic polynomial
trajectory4 = polyval(c4, t);
c4d = polyder(c4);
velocity4 = polyval(c4d, t);
c4dd = polyder(c4d);
acceleration4 = polyval(c4dd, t);
% evaluating trajectory, velocity and acceleration of hexic polynomial
trajectory6 = polyval(c6, t);
c6d = polyder(c6);
velocity6 = polyval(c6d, t);
c6dd = polyder(c6d);
acceleration6 = polyval(c6dd, t);
% evaluating trajectory, velocity and acceleration of two cubic polynomial
trajectory3_via = [polyval(c31, t01), polyval(c32, t1f)];
c31d = polyder(c31);
c32d = polyder(c32);
velocity3_via = [polyval(c31d, t01), polyval(c32d, t1f)];
c31dd = polyder(c31d);
c32dd = polyder(c32d);
acceleration3_via = [polyval(c31dd, t01), polyval(c32dd, t1f)];
% evaluating trajectory, velocity and acceleration of two quintic polynomial
trajectory5_via = [polyval(c51, t01), polyval(c52, t1f)];
c51d = polyder(c51);
c52d = polyder(c52);
velocity5_via = [polyval(c51d, t01), polyval(c52d, t1f)];
c51dd = polyder(c51d);
c52dd = polyder(c52d);
acceleration5_via = [polyval(c51dd, t01), polyval(c52dd, t1f)];

% plots
% plotting angle acceleration for quartic and hexic trajectory
figure(6);
clf;
plot(t, acceleration4, t, acceleration6, t, acceleration3_via, t, acceleration5_via); grid on;
title('Angle acceleration for quartic and hexic trajectory');
xlabel('$t$ in $s$','Interpreter','latex');
ylabel('$\ddot{\theta}$ in $^{\circ}/s^2$','Interpreter','latex');
legend('quartic', 'hexic', 'two cubic', 'two quintic');

% plotting angle velocity for quartic and hexic trajectory
figure(5);
clf;
plot(t, velocity4, t, velocity6, t, velocity3_via, t, velocity5_via); grid on;
title('Angle velocity for quartic and hexic trajectory');
xlabel('$t$ in $s$','Interpreter','latex');
ylabel('$\ddot{\theta}$ in $^{\circ}/s^2$','Interpreter','latex');
legend('quartic', 'hexic', 'two cubic', 'two quintic');

% plotting angle position for quartic and hexic trajectory
figure(4);
clf;
plot(t, trajectory4, t, trajectory6, t, trajectory3_via, t, trajectory5_via); grid on;
title('Angle position for quartic and hexic trajectory');
xlabel('$t$ in $s$','Interpreter','latex');
ylabel('$\ddot{\theta}$ in $^{\circ}/s^2$','Interpreter','latex');
hold on;
plot(t1, theta1, 'kx', 'MarkerSize', 10, 'LineWidth', 2);
legend('quartic', 'hexic', 'two cubic', 'two quintic');
