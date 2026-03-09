function X_next = robot_step(X, u, dt)

    %X : Current Robot State [x ; y ; theta]
    %u : Control Input [v ; Omega]

    %Extracing current positional state
    x = X(1);
    y = X(2);
    theta = X(3);
    

    %Extracting current linear and angular speed
    v = u(1);
    omega = u(2);
    
    
    %Calculating next state
    x_next = x + v*cos(theta)*dt;
    y_next = y + v*sin(theta)*dt;
    theta_next = wrapToPi(theta + omega*dt);

    %Storing new position values
    X_next = [x_next; y_next; theta_next];

    
end