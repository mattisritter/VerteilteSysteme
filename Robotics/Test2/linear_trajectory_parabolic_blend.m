%% Linear trajectory with parabolic blend
clear all;
%% Parameters
theta1 = 50;
theta2 = 100;
theta3 = 10;

td12 = 2;
td23 = 4;

%% Calculations
acc1 = sign(theta2-theta1) * 100;
t1 = td12 - sqrt(td12^2 - 2 * (theta2-theta1) / acc1);

acc3 = sign(theta3-theta2) * 50;
t3 = td23 - sqrt(td23^2 - 2 * (theta3-theta2) / acc3);

vel12 = (theta2-theta1) / (td12-t1/2);
vel23 = (theta3-theta2) / (td23-t3/2);
acc2 = sign(vel23-vel12) * 75;
t2 = (vel23-vel12) / acc2;
%t2 = 2 * min([td12-t1, td23-t3]);
%acc2 = (vel23-vel12) / t2;
t12 = td12 - t1 - t2/2;
t23 = td23 - t3 - t2/2;

%% Trajectory
ts = 1e-4;
tp1 = 0:ts:t1;
c1 = parabolic_blend(0, t1, theta1, NaN, 0, vel12);
p1 = polyval(c1, tp1);
tp2 = td12-t2/2:ts:td12+t2/2;
c2 = parabolic_blend(td12-t2/2, td12+t2/2, p1(end)+t12*vel12, NaN, vel12, vel23);
p2 = polyval(c2, tp2);
tp3 = td12+td23-t3:ts:td12+td23;
c3 = parabolic_blend(td12+td23-t3, td12+td23, NaN, theta3, vel23, 0);
p3 = polyval(c3, tp3);

%% Visulization
figure(1);
clf;
hold on;
grid on;
plot(0, theta1, 'kx', 'MarkerSize', 10, 'LineWidth', 2);
plot(td12, theta2, 'kx', 'MarkerSize', 10, 'LineWidth', 2);
plot(td12+td23, theta3, 'kx', 'MarkerSize', 10, 'LineWidth', 2);
plot(tp1, p1, 'b');
plot(tp2, p2, 'b');
plot(tp3, p3, 'b');
plot([tp1(end), tp2(1)], [p1(end), p2(1)], 'r');
plot([tp2(end), tp3(1)], [p2(end), p3(1)], 'r');
xlabel('time [s]');
ylabel('angle [°]');
title('Linear Trajectroy with Parabolic Blends');
