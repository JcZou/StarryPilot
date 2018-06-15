function new_adrc = adrc_cleso(adrc, u, y, order)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

h = adrc.h;
e = adrc.z1 - y;
es = adrc.s1 - y;

if order == 3
    beta1 = 3*adrc.w;
    beta2 = beta1*adrc.w;
    beta3 = beta2/3*adrc.w;

    beta4 = 3*adrc.w;
    beta5 = beta4*adrc.w;
    beta6 = beta5/3*adrc.w;

    adrc.z1 = adrc.z1 + h*(adrc.z2 - beta1*e);
    adrc.z2 = adrc.z2 + h*(adrc.z3 + adrc.b0*u - beta2*e);
    adrc.z3 = adrc.z3 - h*beta3*e;

    adrc.s1 = adrc.s1 + h*(adrc.s2 - beta4*es);
    adrc.s2 = adrc.s2 + h*(adrc.s3 + adrc.z3 + adrc.b0*u - beta5*es);
    adrc.s3 = adrc.s3 - h*beta6*es;
elseif order == 2
    beta1 = 2*adrc.w;
    beta2 = adrc.w*adrc.w;
    beta3 = 2*adrc.w;
    beta4 = adrc.w*adrc.w;

    adrc.z1 = adrc.z1 + h*(adrc.z2 + adrc.b0*u - beta1*e);
    adrc.z2 = adrc.z2 - h*beta2*e;

    adrc.s1 = adrc.s1 + h*(adrc.s2 + adrc.z2 + adrc.b0*u - beta3*es);
    adrc.s2 = adrc.s2 - h*beta4*es;
end

new_adrc = adrc;

end

