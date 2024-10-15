%Nick Hauger 9OCT24
%
% ZED resolution: 640x360 (16:9) width x height
% This only works if you have 4:3 aspect original image coming in.
% but we can edit it to work with any image by measuring the ratio.
% This only works on .jpgs right now but I'm sure we can mess with it to do
% png. Or just use img magick to convert pngs to jpgs.
clear variables; clc;

descol = 640; % desired row and desired column size
desrow = 360;

%This is the part where we grab the current directory and the files within
D = dir(pwd); %dir gives you the files in the directory you called
% pwd gives you the directory the .m file is inside.

i = length(D); % we'll check all the files in the directory to see if they're jpgs
j = 1;
while j <= i
    if endsWith(D(j).name,".jpg") %if the file ends in .jpg
        orig = imread(D(j).name); %create an array of data from the jpg
        
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

        s = erase(D(j).name, '.jpg'); %get rid of '.jpg' from the string (especially if theres multiple)
        s = strcat(s,'_resized.jpg'); % append _'resized.jpg' onto the end
        imwrite(combo, s) % and we write it.

    end
    j = j+1; %check the next file in the current folder
end

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
