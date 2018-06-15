function new_adrc = adrc_td(adrc, v)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

h = adrc.h;
fv = fhan(adrc.v1 - v, adrc.v2, adrc.r0,  h);
% fv = -adrc.r0*(adrc.r0*(adrc.v1-v)+sqrt(3)*adrc.v2);
adrc.v1 = adrc.v1 + h*adrc.v2;
adrc.v2 = adrc.v2 + h*fv;

new_adrc = adrc;

end

