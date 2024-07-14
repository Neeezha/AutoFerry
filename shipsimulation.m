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
% initializing the position vectors
x=zeros(N,1); y=zeros(N,1); theta = zeros(N,1);
% initializing error vectors
x_err=zeros(N,1); y_err=zeros(N,1); theta_err = zeros(N,1);
% initializing desired position vectors
x_des=zeros(N,1); y_des=zeros(N,1); % theta_des = zeros(N,1);
% initializing rate of change vectors
x_dot = zeros(N,1); y_dot = zeros(N,1); theta_dot = zeros(N,1);
% initializing heading control vector
u = zeros(N,1); 

% Constant values
V = 7; % We'll start with a constant velocity
        % this should be adjusted alongside b
b = 2.5;  % we'll keep the variable b as a constant at first
        % b affects rotation rate of change, should be changed alongside V
Kp = 1; % Proportional control gain, can be adjusted after for-loop works

% Initial position and angle parameters
theta(1) = 135;
x(1) = 98; y(1) = 83; 
x_des(1) = 80; y_des(1) = 88;

% theta(1) = 45; % These were the original inital position/heading values
% x(1) = 80; y(1) = 20;% we start at (80,20)
% % Initial desired position parameters
% x_des(1) = 85; y_des(1) = 30; % and the first point we want to head
% towards is (85,30)

% Hard coded waypoints for ferry to follow 
% 8 rows, 2 columns; x_n and y_n respectively
% New waypoints to follow the ferry path on the background map
next_wp = [73 90; 64 93; 49 93; 43 82; 31 55; 22 34; 15 17; 3 14;
            13 13; 23 36; 35 64; 44 87; 51 94; 74 89; 97 84];
% Original waypoints before we used a background map
% next_wp = [87 40; 80 45; 75 50; 65 55; 60 60; 50 70; 45 75; 30 90;
%     25 75; 15 65; 25 55; 30 50; 40 40; 50 35; 60 30; 70 25; 80 20];

% Counter for waypoints reached, starting at 1
way_index = 1; % Increment each time each waypoint is reached
next_wp_size = length(next_wp); % Checks length of next_wp array

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
    
    % Waypoint generator, based on distance from current waypoint
    dist = sqrt(x_err(k)^2 + y_err(k)^2);
    
    % checks if boat is within range of the waypoint
    % Sets a new waypoint if within range of current wp, and we arent on
    % the final index of next_wp vector
    if dist < 2 && way_index < next_wp_size 
        % Set next destination to 
        way_index = way_index + 1;
        x_des(k) = next_wp(way_index, 1);
        y_des(k) = next_wp(way_index, 2);

    elseif way_index == next_wp_size %if we reach wp end, loop to index 1
        way_index = 1;
        x_des(k) = x_des(k-1);
        y_des(k) = y_des(k-1);
        theta(k-1) = theta(k-1) - 360;
        
    % Prepare next waypoint
    else 
        x_des(k) = x_des(k-1);
        y_des(k) = y_des(k-1);
    end

    % using error margins to find desired angle to rotate
    theta_des = atan2d(y_err(k), x_err(k));

    if theta(k) > 180 % this keeps our current theta between -180 & 180
        theta(k) = theta(k) - 360;
    end

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

%%%% This section constructs shapes, but no computation is done %%%%

% New reference frame of 0 to 100 range for both axes
figure(1); 
hold on; 
axis equal
xlim([0 100])
ylim([0 100])

%this creates an image object we can manipulate and plot as a background
im = imread('map.png');
backgrnd = image(xlim,flip(ylim),im);
uistack(backgrnd,'bottom')

% This constructs the ship's body using the patch function to make a shape
% we can manipulate
shipbody_x = [1, 0, -1, -1, 0, 1]; % these form the coordinates of our shape
shipbody_y = [0, 0.5, 0.5, -0.5, -0.5, 0];
s = hgtransform;
% This forms the shape, connects it to s
patch('XData', shipbody_x, 'YData', shipbody_y, 'Parent', s) 

% % Draw two lines; these represent landmasses % Not needed with background
% LM_x1 = [0 40]; LM_x2 = [60 100];
% LM_y1 = [70 100]; LM_y2 = [0 30]; 
% line(LM_x1, LM_y1); line(LM_x2, LM_y2);

% Setup trail and waypoint ahead of loop 
ship_trail = plot([x(1) x(2)],[y(1) y(2)],'--'); 
current_waypoint = plot(x_des(1), y_des(1), 'x'); 

%%%% Plotting ship, waypoint, and path %%%%
for i = 1:N-1

    % Plot ship at current location/orientation
    s.Matrix = makehgtform('translate', [x(i+1) y(i+1) 0], 'zrotate', theta(i)*pi/180);
    
    % Update trail of the ship
    set(ship_trail,'XData',x(1:i)); 
    set(ship_trail,'YData',y(1:i)); 

    % Update waypoint plot 
    set(current_waypoint,'XData',x_des(i)); 
    set(current_waypoint,'YData',y_des(i)); 
    
    drawnow % This is what draws the shape every update

end
