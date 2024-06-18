
% Nick Hauger 21MAY24
% Tan Nguyen 06/18/24
% Note: all math is done in degrees
% remember commas seperate columns, semicolons rows
% indexing matrices: A(#,:) gives entire row
                   % A(:, #) gives entire column
                   % A(2:3,#) gives 2nd-3rd rows, #th column
clear variables; clc; close all;
% state:    (x, y , theta, V)
% roc:      (x_dot, y_dot, theta_dot, V_dot)

dt = 0.01; % our time interval
tspan = 0:dt:200; % a finite vector for our time to start out
N = length(tspan); 

% u_1 = cosd(rot90(tspan(1,:))); 
% We'll start with a sinusoid and mess with the control input later
% now redundant due to u being vector now

x=zeros(N,1); % initializing the position vectors
y=zeros(N,1);
theta = zeros(N,1);

x_err=zeros(N,1); % initializing error vectors
y_err=zeros(N,1);
theta_err = zeros(N,1);

x_des=zeros(N,1); % initializing desired position vectors
y_des=zeros(N,1);
% theta_des = zeros(N,1);

x_dot = zeros(N,1); % initializing rate of change vectors
y_dot = zeros(N,1);
theta_dot = zeros(N,1);

u = zeros(N,1); % initializing heading control vector

% Constant values
V = 10; % We'll start with a constant velocity
        % this should be adjusted alongside b
b = 3;  % we'll keep the variable b as a constant at first
        % b affects rotation rate of change, should be changed alongside V
        % it was at the top of the program with timescale variables
Kp = 1; % Proportional control gain, can be adjusted after for-loop works

% Initial position and angle parameters
theta(1) = 160;
x(1) = 10;
y(1) = 5;

% Initial desired position parameters
x_des(1) = 50;
y_des(1) = 50;

% specifying an initial condition
% ship_state(:,1)=0;

% We'll do a loop where we update the equations as we go.
for k = 2:(N)
    % our rate of change vectors for current loop
    x_dot(k) = V.*cosd(theta(k-1)); % derivative of x
    y_dot(k) = V.*sind(theta(k-1)); % derivative of y
    theta_dot(k) = b*(u(k-1)); % derivative of theta

    % calculating the next point based upon the current + the derivative 
    theta(k) = theta(k-1) + theta_dot(k)*dt;
    x(k) = x(k-1) + x_dot(k)*dt;
    y(k) = y(k-1) + y_dot(k)*dt;
    
    % finding error margin between points
    x_err(k) = x_des(k-1) - x(k);
    y_err(k) = y_des(k-1) - y(k);

    % Debugging for variable values
    % x_err(k)
    % x_des(k)
    % x(k)

    % Waypoint generator, based on distance from current waypoint
    dist = sqrt(x_err(k)^2 + y_err(k)^2);
    % checks if boat is within range of the waypoint
    if dist < 2 % Set new waypoint if within range
        x_des(k) = rand(1)*100;
        y_des(k) = rand(1)*100;
        % x_err(k) = x_des(k) - x(k);
        % y_err(k) = y_des(k) - y(k);

    % Prepare next waypoint
    % It works for now, but causes the current waypoint to flash
    else 
        x_des(k) = x_des(k-1);
        y_des(k) = y_des(k-1);
    end

    % using error margins to find desired angle to rotate
    theta_des = atan2d(y_err(k), x_err(k));

    % Adjust for desired turn direction
    % If turning in one direction is > 180 degress, go the other direction
    if ((theta_des - theta(k)) > 180)
        theta_des = theta_des - 360;
    elseif (theta_des - theta(k) < -180)
        theta_des = theta_des + 360;
    end

    % calculating angle error for heading control
    theta_err(k) = theta_des - theta(k);

    % Calculate heading control input
    u(k) = Kp * theta_err(k);
end 

% This constructs the ships body using the patch function to make a shape
% we can manipulate
shipbody_x = [1, 0, -1, -1, 0, 1]; % these form the coordinates of our shape
shipbody_y = [0, 0.5, 0.5, -0.5, -0.5, 0];
s = hgtransform;
% This forms the shape, connects it to s
patch('XData', shipbody_x, 'YData', shipbody_y, 'Parent', s) 

% New reference frame of 0 to 100 range for both axes
axis equal
xlim([0 100])
ylim([0 100])

% Plots ship, waypoint, and path
for i = 1:N-1
    hold on;
    s.Matrix = makehgtform('translate', [x(i+1) y(i+1) 0], 'zrotate', theta(i)*pi/180);
    %drawnow
    %plot(x(i),y(i),'o')
    plot([x(i) x(i+1)],[y(i) y(i+1)],'--')

    % Plots waypoint
    plot(x_des(i), y_des(i), 'x')

    drawnow % I think this is what draws the shape every update
    hold off;
end
