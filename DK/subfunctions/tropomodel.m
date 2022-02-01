function [tropo] = tropomodel(el, zd)

Tz = zd;

for i = 1:length(el)
    md= 1/sind(el(i));
    tropo(i) = Tz * md;
end

tropo = tropo.';

end