function u = adrc_nlsef(adrc, z1, z2)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

% e1 = adrc.v1 - adrc.z1;
% e2 = adrc.v2 - adrc.z2;
e1 = adrc.v1 - z1;
e2 = adrc.v2 - z2;
u = -fhan(e1, adrc.c*e2, adrc.r, adrc.h1)/adrc.b0;
% u0 = 20*e1+5*e2;

% u = (u0-adrc.z3)/adrc.b0;
% u = u0-adrc.z3/adrc.b0;

end

