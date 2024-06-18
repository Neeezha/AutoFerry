% Nick Hauger 21MAY24
%remember commas seperate columns, semicolons rows
%indexing matrices: A(#,:) gives entire row
                  %  A(:, #) gives entire column
                   % A(2:3,#) gives 2nd-3rd rows, #th column
clear variables; clc; close all;
% state: (x, y , theta, V)
% roc: (x_dot, y_dot, theta_dot, V_dot)

b = 10; % we'll keep the variable b as a constant at first
dt = 0.01; % our time interval
tspan = 0:dt:200; % a finite vector for our time to start out
N = length(tspan);
u_1 = cosd(rot90(tspan(1,:))); % We'll start with a sinusoid and mess with the control input later

x=zeros(N,1); %initializing the position vectors
y=zeros(N,1);
theta = zeros(N,1);
V = 1; % We'll start with a constant velocity

theta(1) = 0; %start at 0 degrees

%specifying an initial condition
ship_state(:,1)=0;

% We'll do a loop where we update the equations as we go.
for k = 1:(N-1)
    % our rate of change vectors
    x_dot = V.*cosd(theta); %derivative of x
    y_dot = V.*sind(theta); % derivative of y
    theta_dot = b*(u_1); % derivative of theta

    % calculating the next point based upon the current + the derivative 
    theta(k+1) = theta(k) + theta_dot(k)*dt;
    x(k+1) = x(k) + x_dot(k)*dt;
    y(k+1) = y(k) + y_dot(k)*dt;
end 

% This constructs the ships body using the patch function to make a shape
% we can manipulate
shipbody_x = [1, 0, -1, -1, 0, 1]; % these form the coordinates of our shape
shipbody_y = [0, 0.5, 0.5, -0.5, -0.5, 0];
s = hgtransform;
patch('XData', shipbody_x, 'YData', shipbody_y, 'Parent',s) %this forms the shape, connects it to s

axis equal
xlim([-10 10])
ylim([-10 10])
for i = 1:N-1
    hold on;
    s.Matrix = makehgtform('translate', [x(i+1) y(i+1) 0], 'zrotate', theta(i)*pi/180);
    %drawnow
    %plot(x(i),y(i),'o')
    plot([x(i) x(i+1)],[y(i) y(i+1)],'--')
    drawnow % I think this is what draws the shape every update
    hold off;
end


% more advanced Update equations
% 
% y_err = y_dest - y_current;
% x_err = x_dest - x_current;
% dtheta = atan2(y_err/ x_err)
% 
% theta_error = dtheta - theta;
% u = Kp * theta_error;

