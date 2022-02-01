function [ ymd ] = gps2cal(wn,tow )
jd = gps2jd(wn,tow);
[y,m,d]=jd2cal(jd);
ymd = [y m d];
end

