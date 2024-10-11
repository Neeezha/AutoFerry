%Nick Hauger 9OCT24
%
% ZED resolution: 640x360 (16:9) width x height
% This only works if you 
clear variables; clc;
descol = 640; % desired row and desired column size
desrow = 360;

% this is just what we wrote on the board.
orig = imread("test.jpg");

A2_scaled = imresize(orig,[desrow, NaN]);

A2_red = A2_scaled(:,:,1); 

pad = zeros(desrow,descol-length(A2_red(1,:)));

A2_comb_red(1:desrow,1:length(A2_red(1,:))) = A2_red;
A2_comb_red = [A2_comb_red,pad];
imshow(A2_comb_red)


% %%%%%%%% Unneeded %%%%%%%
% edit = orig; %making a copy of the original image called edit
% %this flips the image if its portait into landscape.
% if col < row
%     edit = imrotate(edit, 90); %transpose the edit matrix
%     temp = row; row = col; col = temp; %swap row and col (theyre #s)
% end
% 
% rratio = row/desrow; % ratio of img's rows to desired rows
% cratio = col/descol; % ratio of img's cols to desired cols
% if rratio < cratio
%     resized = imresize(edit, [descol, NaN]);
% else 
%     resized = imresize(edit, [NaN, desrow]);
% end
