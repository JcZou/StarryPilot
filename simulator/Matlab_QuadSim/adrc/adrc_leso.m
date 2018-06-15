function new_adrc = adrc_leso(adrc, u, y)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

h = adrc.h;
e = adrc.z1 - y;
beta1 = 3*adrc.w;
beta2 = beta1*adrc.w;
beta3 = beta2/3*adrc.w;

adrc.z1 = adrc.z1 + h*(adrc.z2 - beta1*e);
adrc.z2 = adrc.z2 + h*(adrc.z3 + adrc.b0*u - beta2*e);
adrc.z3 = adrc.z3 - h*beta3*e;

new_adrc = adrc;

end