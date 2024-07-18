


% New reference frame of 0 to 100 range for both axes
figure(1); 
hold on; 

% axis([-1322.1 3188.6 -1849.1 2329.8])


% axis equal
% xlim([0 100])
% ylim([0 100])
axis([-1322.1 3198.6 -1769.1 2320.8])
set(gcf, 'Position',  [100, 50, 704, 630])

%this creates an image object we can manipulate and plot as a background
im = imread('map.png');
backgrnd = image(xlim,flip(ylim),im);
uistack(backgrnd,'bottom')

plot(136.1, -28.7, '*')
plot(166.3, -28.0, '*')
plot(196.7, -18.7, '*')
plot(228.2, 12.12, '*')

plot(895.0, 1290.0, '*')
plot(946.3, 1327.3, '*')
plot(1010.5, 1339.7, '*')
plot(1226.3, 1332.4, '*')

plot(1927.3, 1179.9, '*')

% Bottom left (-1322.1, -1849.1)
% Top right (3284.6, 2418.7)

% Top right correction (3188.6, 2329.8)

plot(0, 0, '*')