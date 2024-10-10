%Nick Hauger 9OCT24
%
% ZED resolution: 640x360 (16:9) width x height
clear variables; clc;
desrow = 640; % desired row and desired column size
descol = 360;

% this is just what we wrote on the board.
orig = imread("test.jpg");
row = size(orig,1); % note: size returns [row, col, # of color channels]
col = size(orig,2);

edit = orig; %making a copy of the original image called edit
%this flips the image if its portait into landscape.
if col < row
    edit = imrotate(edit, 90); %transpose the edit matrix
    temp = row; row = col; col = temp; %swap rol and col (theyre #s)
end

red = edit(1:row,1:col,1); % col 3 is 1 - the red channel

imshow(red) % this is just checking it works

rratio = row/desrow % ratio of img's rows to desired rows
cratio = col/descol % ratio of img's cols to desired cols
if rratio > cratio
    resized = imresize(edit, [360, NaN]);
else 
    resized = imresize(edit, [NaN, 640]);
end

imwrite(resized,'r.jpg')