clc 
clear
close all

dt = 0.05;
T = 20;

time = 0:dt:T

%Initital Positional State
x0 = 0;
y0 = 0;
theta_deg = 0;
theta0 = deg2rad(theta_deg);


current_pos = [x0;y0;(theta0)];

%Initial Control Inputs

v0 = 10;
omega_deg = 90;   % degrees/sec , Pos Values: CCW

omega = deg2rad(omega_deg); 

u = [v0, omega];

trajectory = zeros(3,length(time));
trajectory(:, 1) = current_pos;
R = zeros(1, length(time));

for k = 1 : length(time)-1

    current_pos = robot_step(current_pos, u, dt);
    R(k+1) = sqrt(current_pos(1)^2 + current_pos(2)^2);

    trajectory(:, k+1) = current_pos;
    

end

disp('R = ')
disp(R)
disp('trajectory = ');
disp(trajectory);

x_plot = trajectory(1,:);
y_plot = trajectory(2,:);

%plotting y pos wrt x pos
figure
plot(x_plot,y_plot,'LineWidth',2)
grid on
axis equal

xlabel('x position (m)')
ylabel('y position (m)')
title('Robot position')
legend('bot')

%Plotting resultant pos wrt time
figure
plot(time,R,"ro -", 'LineWidth',1)
grid on
axis equal

xlabel('time')
ylabel('Resultant position (magnitude)')
title('Robot Movement with time')

