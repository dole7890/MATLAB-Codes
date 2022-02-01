function C_ECEF2ENU = ECEF2ENU(lat_deg,lon_deg)

% phi/lambda for latitude/longitude
p = lat_deg;
l = lon_deg;

% Rotation Matrix ECEF -> ENU
C_ECEF2ENU = [
    -sind(l), cosd(l), 0;
    -sind(p)*cosd(l), -sind(p)*sind(l), cosd(p);
    cosd(p)*cosd(l), cosd(p)*sind(l), sind(p)];
end