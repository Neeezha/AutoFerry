function [heading,magnitude] = wind_grabber()
%WIND_GRABBER Grabs and outputs the wind data for Ketron Island
%   This function takes the wind data from
%   https://wind.willyweather.com/wa/pierce-county/ketron-island.html 
% and grabs the data closest to the utc timestamp of the webpage, then
% sends this data out as its output in an array. 

% webread grabs the plaintext webpage data. Don't run this function too many times a second or we'll be DDoSing them 
pgdata = webread("https://wind.willyweather.com/wa/pierce-county/ketron-island.html");

% in this section we grab the webpage's unix timecode and manipulate it so
% its the same size as the other timecode's we'll be comparing it against
% to grab our data.
utc_date = extractBetween(pgdata,'utcTimestamp = ', ';'); % the webpage's unix timecode
utc_date = char(utc_date); % convert to a char array so I can remove the last 2 indices
utc_date = utc_date(1:10); % remove the last 2 indices
utc_date = string(utc_date); % convert back into string

%%% grabbing forecast data. Trying to get as close to date as possible %%%
forecast = extractBetween(pgdata,'data: {"f','}}}}}}','Boundaries','inclusive');
forecast = split(forecast,'},{');
% I grab the first 90 lines because: 
% 1.) I run into issues with multiple indexes per extraction the more I grab
% 2.) From my experience, the closest forecast date is near the beginning
fcast_date = extractBetween(forecast(1:90),'"x":',','); 
for i = 1:90
    if utc_date <= fcast_date(i) %comapare the strings
        cls_date_ind = i - 1; % grab the date right before webpage's utc
        break; % pop out of the loop
    end
end % this could be implemented better. 
% ex. compare > and < values in relation to wbpg utc and grab the closer one

% now that we know the closest date, we can grab the information we want
% from that time index. We can create more output arguments the more data we
% want to send to the simulation.
% cls_info = forecast(cls_date_ind);
% cls_date = fcast_date(cls_date_ind);
% cls_date_heading = extractBetween(cls_info,'"direction":',',');
% cls_date_desc = extractBetween(cls_info,'"description":"','"');
% cls_date_mph = extractBetween(cls_info,'"y":',',');

%%% grabbbing the real-time observational data %%%
real = extractBetween(pgdata,'"observationalGraphs":','}}}}}}','Boundaries','inclusive');
real = split(real,'},{'); % splitting the data into its components
real = real(length(real)); % grabbing the final index of data (the closest to utc)
real = extractBefore(real, '}],"controlPoints":');
% now that we know the closest date, we can grab the information we want
% from that time index. We can create more output arguments the more data we
% want to send to the simulation.
cls_date = extractBetween(real,'"x":',',');
cls_date_heading = extractBetween(real,'"direction":',',');
cls_date_desc = extractBetween(real,'"description":"','"');
cls_date_mph = extractBetween(real,'"y":',',');

% Our final outputs to the main function. We could output the variables
% we're setting them equal to, but this just feels cleaner to me?
heading = double(string(cls_date_heading)); % output 1
magnitude = double(string(cls_date_mph)); % output 2
end