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

dt = 0.1; % our time interval
tspan = 0:dt:1000; % a finite vector for our time to start out
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
V = 10; % We'll start with a constant velocity
        % this should be adjusted alongside b
b = 2.5;  % we'll keep the variable b as a constant at first
        % b affects rotation rate of change, should be changed alongside V
Kp = 1; % Proportional control gain, can be adjusted after for-loop works

% plot(136.1, -28.7, '*')
% plot(166.3, -28.0, '*')
% plot(196.7, -18.7, '*')
% plot(228.2, 12.12, '*')
% 
% plot(895.0, 1290.0, '*')
% plot(946.3, 1327.3, '*')
% plot(1010.5, 1339.7, '*')
% plot(1226.3, 1332.4, '*')
% 
% plot(1927.3, 1179.9, '*')

% Hard coded waypoints for ferry to follow 
% variable rows, 2 columns; x_n and y_n respectively
% New waypoints to follow the ferry path on the background map
next_wp = [0 0; 136.1 -28.7; 166.3 -28.0; 196.7 -18.7; 228.2, 12.12;
            895.0 1290.0; 946.3 1327.3; 1010.5 1339.7; 
            1226.3, 1332.4; 1927.3 1179.9; 1226.3, 1332.4;
            1010.5 1339.7; 946.3 1327.3; 895.0 1290.0;
            228.2, 12.12; 196.7 -18.7; 166.3 -28.0; 136.1 -28.7; 0 0];
% Initial position and angle parameters
theta(1) = 135;
x(1) = next_wp(1,1); y(1) = next_wp(1,2); 
x_des(1) = next_wp(2,1); y_des(1) = next_wp(2,2);

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

% New reference figure with fixed axes
figure(1); 
hold on; 
% axis equal
% xlim([0 100])
% ylim([0 100])
%set(gcf, 'Position',  [100, 100, 704, 630])

axis([-1322.1 3198.6 -1769.1 2320.8]) %this sets the figure's axes (in meters)
set(gcf, 'Position',  [100, 50, 704, 630]) %rescales the figure so its bigger

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
patch('XData', shipbody_x*25, 'YData', shipbody_y*25, 'Parent', s) 

% Setup trail and waypoint ahead of loop 
ship_trail = plot([x(1) x(2)],[y(1) y(2)],'--'); 
current_waypoint = plot(x_des(1), y_des(1), 'x'); 

%%%% Plotting ship, waypoint, and path %%%%
for i = 1:N-1

    % Plot ship at current location/orientation
    % the translate moves the figure to the next x,y,z coords
    % the zrotate rotates the figure about the z axis in radians
    s.Matrix = makehgtform('translate', [x(i+1) y(i+1) 0], 'zrotate', theta(i)*pi/180);
    
    % Update trail of the ship
    set(ship_trail,'XData',x(1:i)); 
    set(ship_trail,'YData',y(1:i)); 

    % Update waypoint plot 
    set(current_waypoint,'XData',x_des(i)); 
    set(current_waypoint,'YData',y_des(i)); 
    
    drawnow % This is what draws the shape every update

end
