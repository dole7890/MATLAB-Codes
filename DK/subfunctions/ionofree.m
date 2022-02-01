function iono_free = ionofree(L1,L2,f1,f2)
iono_free = (f1^2*L1 - f2^2*L2)/(f1^2-f2^2);
end