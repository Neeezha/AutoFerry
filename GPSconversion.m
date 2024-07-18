
% Ketron Dock
% 47.162247, -122.629286

% 47.161989, -122.627486

lat1 = 47.162247*pi/180;
lat2 = 47.183999*pi/180;

lon1 = -122.629286*pi/180;
lon2 = -122.585841*pi/180;

delta_lat = lat2 - lat1;
delta_lon = lon2 - lon1;

% a = sin(delta_lat / 2)^2 + cos(lat1)*cos(lat2)*sin(delta_lon / 2)^2;
% 
% c = 2*atan2(sqrt(a), sqrt(1-a));
% 
% R = 6371000;
% d = R*c

a1 = sin(0 / 2)^2 + cos(lat1)*cos(lat1)*sin(delta_lon / 2)^2;

c1 = 2*atan2(sqrt(a1), sqrt(1-a1));

R = 6371000;
x = R*c1

a2 = sin(delta_lat / 2)^2 + cos(lat1)*cos(lat2)*sin(0 / 2)^2;

c2 = 2*atan2(sqrt(a2), sqrt(1-a2));

R = 6371000;
y = R*c2

% First waypoint (136.1, -28.7)
% Second waypoint (166.3, -28.0)
% Third waypoint (196.7, -18.7)
% Fourth (228.2, 12.12)

% Fifth (895.0, 1290.0)
% Sixth (946.3, 1327.3)
% Seventh (1010.5, 1339.7)
% Eighth (1226.3, 1332.4)

% Final (1927.3, 1179.9)

% Bottom left (-1322.1, -1849.1)
% Top right (3284.6, 2418.7)
