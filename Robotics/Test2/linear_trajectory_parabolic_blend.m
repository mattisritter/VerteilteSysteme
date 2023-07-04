%% Linear trajectory with parabolic blend
clear all;
%% Parameters
theta = [80 5 40]; % path points
t = [0 3 5]; % timing for path points
acc = [25 25 25]; % acceleration 
td = diff(t); % time differnce between path points
n = 1000; % number of segments of parabolic blend
% predefinition of variables
vel = zeros(size(td));
tl = zeros(size(td));
tb = zeros(size(theta));
tbs = zeros(length(theta), n);
c = zeros(length(theta), 3);
cv = zeros(length(theta), 2);
p = zeros(size(tbs));
v = zeros(size(p));
aq = zeros(size(theta));

%% Calculations
% first segment
acc(1) = sign(theta(2)-theta(1)) * acc(1);
tb(1) = td(1) - sqrt(td(1)^2 - 2 * (theta(2)-theta(1)) / acc(1));
tbs(1,:) = linspace(t(1), t(1)+tb(1), n);
vel(1) = (theta(2)-theta(1)) / (td(1)-tb(1)/2);
% last segment
acc(end) = sign(theta(end-1)-theta(end)) * acc(end);
tb(end) = td(end) - sqrt(td(end)^2 - 2 * (theta(end-1)-theta(end)) / acc(end));
tbs(end,:) = linspace(t(end)-tb(end), t(end), n);
vel(end) = (theta(end)-theta(end-1)) / (td(end)-tb(end)/2);
% interior segments
for k = 2:length(vel)-1
    vel(k) = (theta(k+1)-theta(k)) / td(k);
end
for k = 2:length(acc)-1
    acc(k) = sign(vel(k)-vel(k-1)) * acc(k);
    tb(k) = (vel(k)-vel(k-1)) / acc(k);
    tbs(k,:) = linspace(t(k)-tb(k)/2, t(k)+tb(k)/2, n);
end
% blend time interior segments
for k = 2:length(tl)-1
    tl(k) = td(k) - tb(k)/2 - tb(k+1)/2;
end
% blend time first segment
tl(1) = td(1) - tb(1) - tb(2)/2;
% blend time last segment
tl(end) = td(end) - tb(end) - tb(end-1)/2;
%% Trajectory
c(1,:) = parabolic_blend(t(1), t(1)+tb(1), theta(1), 0, vel(1));
p(1,:) = polyval(c(1,:), tbs(1,:));
cv(1,:) = polyder(c(1,:));
v(1,:) = polyval(cv(1,:), tbs(1,:));
for k = 2:length(theta)-1
    c(k,:) = parabolic_blend(t(k)-tb(k)/2, t(k)+tb(k)/2, p(k-1,end)+vel(k-1)*tl(k-1), vel(k-1), vel(k));
    p(k,:) = polyval(c(k,:), tbs(k,:));
    cv(k,:) = polyder(c(k,:));
    v(k,:) = polyval(cv(k,:), tbs(k,:));
end
c(end,:) = parabolic_blend(t(end)-tb(end), t(end), p(end-1,end)+vel(end)*tl(end), vel(end), 0);
p(end,:) = polyval(c(end,:), tbs(end,:));
cv(end,:) = polyder(c(end,:));
v(end,:) = polyval(cv(end,:), tbs(end,:));

%% Visulization
disp('Results:');
figure(1);
clf;
hold on;
grid on;
for k = 1:length(theta)
    plot(t(k), theta(k), 'kx', 'MarkerSize', 10, 'LineWidth', 2);
    plot(tbs(k,:), p(k,:), 'b');
    disp(['Blend Time t', num2str(k), ': ', num2str(tb(k))]);
    disp(['Acelleration a', num2str(k), ': ', num2str(acc(k))]);
end
for k = 1:length(td)
    plot([tbs(k,end), tbs(k+1,1)], [p(k,end), p(k+1,1)], 'r');
    disp(['Linear Time t', num2str(k), num2str(k+1), ': ', num2str(tl(k))]);
    disp(['Velocity v', num2str(k), num2str(k+1), ': ', num2str(vel(k))]);
end
xlabel('time [s]');
ylabel('angle [°]');
title('Linear Trajectroy with Parabolic Blends');

figure(2);
clf;
hold on;
grid on;
for k = 1:length(theta)
    plot(tbs(k,:), v(k,:), 'b');
end
for k = 1:length(td)
    plot([tbs(k,end), tbs(k+1,1)], [vel(k), vel(k)], 'r');
end
xlabel('time [s]');
ylabel('velocity [°/s]');
title('Linear Trajectroy with Parabolic Blends');

figure(3);
clf;
hold on;
grid on;
for k = 1:length(theta)
    aq(k) = (v(k,end)-v(k,1)) / (tbs(k,end)-tbs(k,1));
    plot([tbs(k,1), tbs(k,end)], [aq(k), aq(k)], 'b');
end
for k = 1:length(td)
    plot([tbs(k,end), tbs(k+1,1)], [0, 0], 'r');
    plot([tbs(k,end), tbs(k,end)], [aq(k), 0], 'k--');
    plot([tbs(k+1,1), tbs(k+1,1)], [0, aq(k+1)], 'k--');
end
xlabel('time [s]');
ylabel('acceleration [°/s^2]');
title('Linear Trajectroy with Parabolic Blends');

disp(' ');