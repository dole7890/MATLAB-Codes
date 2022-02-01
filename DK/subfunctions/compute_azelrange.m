function [AZ, EL, RANGE] = compute_azelrange(userECEF, satECEF)
% satECEF must be in column vectors [X;Y;Z]

for i = 1:size(satECEF,2)
    LOS_ENU = compute_LOS_ENU(userECEF, satECEF(:,i));
    AZ(i) = atan2d(LOS_ENU(1),LOS_ENU(2));
    EL(i) = asind(LOS_ENU(3)/(norm(LOS_ENU)));
    RANGE(i) = norm(satECEF(:,i) - userECEF);
end

end