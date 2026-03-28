omega0 = 90;
v0 = 10;
theta0 = 0;
dt = 0.05;

traj = out.trajectory.Data

x = traj(1,:);
y = traj(2,:);
theta = traj(3,:);

cx = mean(x);
cy = mean(y);

R = mean(sqrt((x-cx).^2 + (y-cy).^2))

plot(x,y,'o-')
axis equal
grid on
xlabel('x')
ylabel('y')
title('Robot trajectory')



%% Triangle animation

for k = 1:length(x)

    clf
    plot(x(1:k),y(1:k),'b')
    hold on
    
    % robot heading arrow
    quiver(x(k),y(k),cos(theta(k)),sin(theta(k)),0.5,'r','LineWidth',3)
    
    axis equal
    grid on
    xlim([min(x)-2 max(x)+2])
    ylim([min(y)-2 max(y)+2])
    
    drawnow
end
