function LOS_ENU = compute_LOS_ENU(userECEF, satECEF)
% satECEF must be in a column vector [X;Y;Z]

% Vector Pointing from User to Satellite
vecECEF = satECEF - userECEF;
% Convert vector to unit vector
LOS_ECEF = vecECEF/norm(vecECEF);
% Get lat and long for unit vector
LOS_LLA = ecef2lla(userECEF.'); 
% Covert ENU coordinates
% % % % LOS_ENU = ECEF2ENU(LOS_LLA(1), LOS_LLA(2))*vecECEF;
LOS_ENU = ECEF2ENU(LOS_LLA(1), LOS_LLA(2))*LOS_ECEF;
end