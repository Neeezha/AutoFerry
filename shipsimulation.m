% Nick Hauger 21MAY24
% Tan Nguyen 06/18/24
% Note: all math is done in degrees, MAKE SURE sind and cosd is used, not
% sin and cos when using degrees!!!
% remember commas seperate columns, semicolons rows
% indexing matrices: A(#,:) gives entire row
                   % A(:, #) gives entire column
                   % A(2:3,#) gives 2nd-3rd rows, #th column
clear variables; clc; close all;
% state:    (x, y , theta_motor, V_motor)
% roc:      (x_dot, y_dot, theta_dot, V_dot)

dt = 0.1; % our time interval
tspan = 0:dt:5000; % a finite vector for our time to start out
N = length(tspan); 
% initializing the position vectors
x=zeros(N,1); y=zeros(N,1); theta_motor = zeros(N,1); theta_f = zeros(N,1);
% initializing error vectors
x_err=zeros(N,1); y_err=zeros(N,1); theta_err = zeros(N,1);
% initializing desired position vectors
x_des=zeros(N,1); y_des=zeros(N,1); % theta_des = zeros(N,1);
% initializing rate of change vectors
x_dot = zeros(N,1); y_dot = zeros(N,1); theta_dot = zeros(N,1);
% initializing heading control vector
u = zeros(N,1); 

% Constant values
V_motor = 15; % We'll start with a constant velocity magnitude
        % this should be adjusted alongside b
b = 2.5;  % we'll keep the variable b as a constant at first
        % b affects rotation rate of change, should be changed alongside
        % V_motor
Kp = 1; % Proportional control gain, can be adjusted after for-loop works

% Additional velocity magintudes and direction
V_c = 5; theta_c = 60; % water current mag and dir
V_w = 5; theta_w = 90; % wind mag and dir
    % direction is constant rn but we could do a rand fn and make it change

% Hard coded waypoints for ferry to follow 
% variable rows, 2 columns; x_n and y_n respectively
% New waypoints to follow the ferry path on the background map
next_wp = [0 0; 136.1 -28.7; 166.3 -28.0; 196.7 -18.7; 228.2, 12.12;
            895.0 1290.0; 946.3 1327.3; 1010.5 1339.7; 
            1226.3, 1332.4; 1927.3 1179.9; 1226.3, 1332.4;
            1010.5 1339.7; 946.3 1327.3; 895.0 1290.0;
            228.2, 12.12; 196.7 -18.7; 166.3 -28.0; 136.1 -28.7; 0 0];
% Initial position and angle parameters
theta_motor(1) = 0;
x(1) = next_wp(1,1); y(1) = next_wp(1,2); 
x_des(1) = next_wp(2,1); y_des(1) = next_wp(2,2);

% Counter for waypoints reached, starting at 1
way_index = 1; % Increment each time each waypoint is reached
next_wp_size = length(next_wp); % Checks length of next_wp array

% We'll do a loop where we update the equations as we go.
for k = 2:(N)
    %%% calculating final Velocity value based on adding all velocities %%%
    % converting motor velocity and theta into x and y components
    V_motorx = V_motor*cosd(theta_motor(k-1));
    V_motory = V_motor*sind(theta_motor(k-1));
    % converting water current vel and dir into x and y components
    V_cx = V_c*cosd(theta_c); V_cy = V_c*sind(theta_c);
    % converting wind vel and dir into x and y components
    V_wx = V_w*cosd(theta_w); V_wy = V_w*sind(theta_w);
    
    % This is where we add the different vector components together
    V_fx = V_motorx + V_cx + V_wx; %add other x vectors as we make them
    V_fy = V_motory + V_cy + V_wy; %add other y vectors when we make them
    
    % converting back to final V magnitude and theta
    V_f = sqrt(V_fx^2 + V_fy^2);
    theta_f(k) = atan2d(V_fy, V_fx);

    %%% moving on from velocity, we can calc rate of change %%%
    % our rate of change vectors for current loop
    x_dot(k) = V_f.*cosd(theta_f(k)); % derivative of x
    y_dot(k) = V_f.*sind(theta_f(k)); % derivative of y
    theta_dot(k) = b*(u(k-1)); % derivative of theta

    % calculating the next point based upon the current + the derivative 
    theta_motor(k) = theta_motor(k-1) + theta_dot(k)*dt;
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
        % if theta_motor(k-1) > 360
        %     theta_motor(k-1) = theta_motor(k-1) - 360;
        % end
    % Prepare next waypoint
    else 
        x_des(k) = x_des(k-1);
        y_des(k) = y_des(k-1);
    end

    % using error margins to find desired angle to rotate
    theta_des = atan2d(y_err(k), x_err(k));

    % This keeps our current ship's theta between -180 & 180
    theta_motor(k) = degree_bounder(theta_motor(k),theta_motor(k));
    %%% this is replaced by the above fn ^
    % if theta_motor(k) > 180 
    %     theta_motor(k) = theta_motor(k) - 360;
    % elseif theta_motor(k) < -180
    %     theta_motor(k) = theta_motor(k) + 360;
    % end

    % Adjust for desired turn direction
    % If turning in one direction is > 180 degress, go the other direction
    theta_des = degree_bounder(theta_des,theta_des - theta_f(k));
    %%% this is replaced by the above fn ^
    % if ((theta_des - theta_f(k)) > 180)
    %     theta_des = theta_des - 360;
    % elseif (theta_des - theta_f(k) < -180)
    %     theta_des = theta_des + 360; 
    % end
    
    % calculating angle error for heading control
    theta_err(k) = theta_des - theta_f(k);
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
    s.Matrix = makehgtform('translate', [x(i+1) y(i+1) 0], 'zrotate', theta_motor(i)*pi/180);
    
    % Update trail of the ship
    set(ship_trail,'XData',x(1:i)); 
    set(ship_trail,'YData',y(1:i)); 

    % Update waypoint plot 
    set(current_waypoint,'XData',x_des(i)); 
    set(current_waypoint,'YData',y_des(i)); 
    
    drawnow % This is what draws the shape every update

end
