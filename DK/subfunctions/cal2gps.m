function [ wn,tow ] = cal2gps(ymd )
% Converts calendar date Y M D to GPS week number and TOW at start of day
% P. Axelrad 9/2018
%  
jd = cal2jd(ymd(:,1),ymd(:,2),ymd(:,3));
[wn,tow]=jd2gps(jd);
end

