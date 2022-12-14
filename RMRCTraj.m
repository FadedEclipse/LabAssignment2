function [ikPath] = RMRCTraj(robot)
robot.model.base = transl(-0.5,0.112,0.3);

%initial guess and robot pose
q1 = [0 -pi/2 deg2rad(79) deg2rad(-125) pi/2 0];


robot.model.animate(q1);

t = 10;             % Total time (s)
deltaT = 0.05;      % Control frequency
steps = t/deltaT;   % No. of steps for simulation
delta = 2*pi/steps; % Small angle change
epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares - determines if robot is near singularity 
W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector - ones for linear velocity and 0.1 for angular velocity

% 1.2) Allocate array data
m = zeros(steps,1);             % Array for Measure of Manipulability
qMatrix = zeros(steps,6);       % Array for joint anglesR
qdot = zeros(steps,6);          % Array for joint velocities
theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
x = zeros(3,steps);             % Array for x-y-z trajectory
positionError = zeros(3,steps); % For plotting trajectory error
angleError = zeros(3,steps);    % For plotting trajectory error

% 1.3) Set up trajectory, initial pose - generates trajectory 
s = lspb(0,1,steps);                % Trapezoidal trajectory scalar - -	is a Cartesian trajectory (4 × 4 × n) from pose 0 to 1 - Allows the robot to get between two points as fast as possible - -	velocity profile takes on the shape of a trapezoid 
for i=1:steps
    x(1,i) = (1-s(i))*-0.710 + s(i)*-0.710; % Points in x
    x(2,i) = (1-s(i))*0 + s(i)*0; % Points in y
    x(3,i) =(1-s(i))*0.855 + s(i)*0.955; % Points in z
    theta(1,i) = 0;                 % Roll angle 
    theta(2,i) = 0;            % Pitch angle
    theta(3,i) = 0;                 % Yaw angle
end
 
T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
q0 = q1;                                                            % Initial guess for joint angles
qMatrix(1,:) = robot.model.ikcon(T,q0);                                 % Solve joint angles to achieve first waypoint - solved using ikcon - Ikcon uses the results of optimisation - It considers joint limits  - Ikcon allowed me to convert my Cartesian trajectory (4 × 4 × n) into its equivalent joint space trajectory (n x 7) 

% 1.4) Track the trajectory with RMRC
for i = 1:steps-1
    T = robot.model.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state - fkine calculates and achieves from known joint angles the required solutions for robots end effector poses.
    deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
    S = Rdot*Ra';                                                           % Skew symmetric!
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!! - skew matrix contains angular velocities 
    deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
    xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
    J = robot.model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
    m(i) = sqrt(det(J*J')); % determines how close the robot is to a singularity - in this case any value returned below 0.1
    if m(i) < epsilon  % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0; % if robot is not near singularity damping is equal to 0
    end
    invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse - avoids singularities 
    qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the  vector)
    for j = 1:6                                                             % Loop through joints 1 to 6 to check validity of joint limits
        if qMatrix(i,j) + deltaT*qdot(i,j) < robot.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
            qdot(i,j) = 0; % Stop the motor if joint limits are close to violation
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
            qdot(i,j) = 0; % Stop the motor if joint limits are close to violation
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
    positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
    angleError(:,i) = deltaTheta;                                           % For plotting
end

% 1.5) Plot the results
figure(1)
plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)

ikPath = qMatrix;


end

