function [PRIF, iono1, iono2] = ionocorr (C1, f1, P2, f2)

PRIF = (f1^2 * C1 - f2^2 * P2)/(f1^2 - f2^2);

TEC = (f1^2*f2^2)/(40.3*(f1^2 - f2^2)) * (P2 - C1);

iono1 = -40.3/(f1^2) * TEC;
iono2 = -40.3/(f2^2) * TEC;




end