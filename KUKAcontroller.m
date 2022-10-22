function KUKAcontroller(kuka)
%% Setup and Initialise joystick
id = 1;
joy = vrjoystick(id);
caps(joy); % display joystick information                

%% Start "real-time" simulation
% This is a version of the lab 11 code that works specifically with our
% robots
qK = [pi/2  pi/4         -pi/4   0    0        0];
q = qK;                

kuka.model.delay = 0.001;    % Set smaller delay when animating

duration = 300;  % Set duration of the simulation (seconds)
dt = 0.15;      % Set time step for simulation (seconds)

n = 0;  % Initialise step count to zero 
tic;    % recording simulation start time
while( toc < duration)
    
    n=n+1; % increment step count

    % read joystick
    [axes, buttons, povs] = read(joy);
       
    % turn joystick input into an end-effector velocity command
    Kv = 0.3; % linear velocity gain
    Kw = 0.8; % angular velocity gain
    
    vx = Kv*axes(1);
    vy = Kv*axes(2);
    vz = Kv*(buttons(5)-buttons(7));
    
    wx = Kw*axes(4);
    wy = Kw*axes(3);
    wz = Kw*(buttons(6)-buttons(8));
    
    dx = [vx;vy;vz;wx;wy;wz]; % combined velocity vector
    
    % use damped least squares Jacobian inverse to calculate joint velocity
    lambda = 0.5;
    J = kuka.model.jacob0(q);
    Jinv_dls = inv((J'*J)+lambda^2*eye(6))*J';
    dq = Jinv_dls*dx;
    
    % apply joint velocity to step robot joint angles 
    q = q + dq'*dt;
    
    % Update plot
    kuka.model.animate(q);  
    
    % wait until loop time elapsed
    if (toc > dt*n)
        warning('Loop %i took too much time - consider increating dt',n);
    end
    while (toc < dt*n); % wait until loop time (dt) has elapsed 
    end
end

end