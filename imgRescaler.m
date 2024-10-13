%Nick Hauger 9OCT24
%
% ZED resolution: 640x360 (16:9) width x height
% This only works if you have 4:3 aspect original image coming in.
% but we can edit it to work with any image by measuring the ratio.
clear variables; clc;
descol = 640; % desired row and desired column size
desrow = 360;

orig = imread("test.jpg"); %create an array of data from the test jpg

[row,col,ignore] = size(orig); % we need to grab ignore or this fn breaks
%this flips the image if its portait into landscape.
if col < row
    orig = imrotate(orig, 90); %flip the portait 90 deg to landscape it
end

 %scale it down to desired rows (assume its in landscape and originally 4:3)
scaled = imresize(orig,[desrow, NaN]);

red = scaled(:,:,1); %grabbing the red matrix values from scaled

%creates a zero-pad vector to fill in the end of our image to make it 640
pad = zeros(desrow,descol-length(red(1,:))); 

% creating a vector with red's values in the first ???x360 positions
% and pad's zero-padding in the last ???+1:640x360 positions.
red = [red,pad]; %vector concatenating pad to the end of red. 
%imshow(red)

% Now repeating for green and blue values.
green = scaled(:,:,2); %grabbing the green matrix values from scaled
green = [green,pad];%vector concatenating pad to the end. 
%imshow(green)

blue = scaled(:,:,3); %grabbing the blue matrix values from scaled
blue = [blue,pad];%vector concatenating pad to the end. 
%imshow(blue)

% Now we combine the values of the three vectors into one, 'combo'.
% Each channel gets its corresponding color vector.
combo(1:desrow,1:descol,1) = red;
combo(1:desrow,1:descol,2) = green;
combo(1:desrow,1:descol,3) = blue;
imshow(combo)
imwrite(combo, 'resized.jpg') %and we write it.
% 
% We'll have to work more to automate this process through an entire folder
% Like, take the original username and overwrite it with resized combo?
% as long as the person running this keeps a copy. of the original.

% %%%%%%%% Unneeded ratio checking. %%%%%%%
%%%%% We will need this if images we use aren't 4:3 %%%%%
% edit = orig; %making a copy of the original image called edit
% 
% rratio = row/desrow; % ratio of img's rows to desired rows
% cratio = col/descol; % ratio of img's cols to desired cols
% if rratio < cratio
%     resized = imresize(edit, [desrow, NaN]);
% else 
%     resized = imresize(edit, [NaN, descol]);
% end
