clc
clear
close all

dt = 0.1;
T = 10;

time = 0:dt:T;

%% TEST 1 — Straight Motion
disp("TEST 1: Straight Motion")

% Initial positional state
x0 = 0;
y0 = 0;
theta_deg = 0;

theta0 = deg2rad(theta_deg);

current_pos = [x0; y0; theta0];

% Control input
v0 = 5;
omega_deg = 0;

omega = deg2rad(omega_deg);

u = [v0; omega];

trajectory = zeros(3,length(time));
trajectory(:,1) = current_pos;

for k = 1:length(time)-1

    current_pos = robot_step(current_pos,u,dt);

    trajectory(:,k+1) = current_pos;

end

figure
plot(trajectory(1,:),trajectory(2,:),'LineWidth',2)
grid on
axis equal
title("Test 1: Straight Motion")
xlabel("x position (m)")
ylabel("y position (m)")


%% TEST 2 — Pure Rotation
disp("TEST 2: Pure Rotation")

x0 = 0;
y0 = 0;
theta_deg = 0;
theta0 = deg2rad(theta_deg);

current_pos = [x0; y0; theta0];

v0 = 0;
omega_deg = 45;
omega = deg2rad(omega_deg);

u = [v0; omega];

trajectory = zeros(3,length(time));
trajectory(:,1) = current_pos;

for k = 1:length(time)-1

    current_pos = robot_step(current_pos,u,dt);

    trajectory(:,k+1) = current_pos;

end

figure
plot(trajectory(1,:),trajectory(2,:),'LineWidth',2)
grid on
axis equal
title("Test 2: Pure Rotation (Robot should stay in place)")
xlabel("x position (m)")
ylabel("y position (m)")


%% TEST 3 — Circular Motion
disp("TEST 3: Circular Motion")

x0 = 0;
y0 = 0;
theta_deg = 0;
theta0 = deg2rad(theta_deg);

current_pos = [x0; y0; theta0];

v0 = 5;
omega_deg = 45;
omega = deg2rad(omega_deg);

u = [v0; omega];

trajectory = zeros(3,length(time));
trajectory(:,1) = current_pos;

for k = 1:length(time)-1

    current_pos = robot_step(current_pos,u,dt);

    trajectory(:,k+1) = current_pos;

end

figure
plot(trajectory(1,:),trajectory(2,:),'LineWidth',2)
grid on
axis equal
title("Test 3: Circular Motion")
xlabel("x position (m)")
ylabel("y position (m)")