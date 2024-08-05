function [degout] = degree_bounder(degin,degcheck)
%DEGREE_BOUNDER This function takes in a number in degrees and bounds it
% between -180 and 180 degrees. - nzh 04AUG24
% degin: the degree being passed in to be manipulated
% degcheck: the degree we are checking is bound by [-180,180].
% degout: the output variable being returned
%   If the number is between -180 and 180 degrees, it returns it
% If it's less than -180 it adds 360 degrees
% If it's less than 180 it adds 360 degrees.
if degcheck > 180
    degout = degin - 360;
elseif degcheck < -180
    degout = degin + 360;
else % if degree passed is between -180 & 180, return it
    degout = degin;
end

