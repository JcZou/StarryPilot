function new_adrc = adrc_eso(adrc, u, y)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

h = adrc.h;
e = adrc.z1 - y;
fe = fal(e, 0.5, h);
fe1 = fal(e, 0.25, h);
%u1 = u + Var.adrc.d;
%u1 = u;
%adrc.b0 = 4*model.L*sin(pi/4)/model.Ixx*(2*model.c1*base_th+model.c2);
% adrc.b0 = 354;

adrc.z1 = adrc.z1 + h*(adrc.z2 - adrc.beta1*e);
% Var.adrc.z2 = Var.adrc.z2 + h*(Var.adrc.z3 + Var.adrc.b0*u1*sqrt(abs(u1)) - Var.adrc.beta2*fe);
adrc.z2 = adrc.z2 + h*(adrc.z3 + adrc.b0*u - adrc.beta2*fe);
adrc.z3 = adrc.z3 - h*adrc.beta3*fe1;

new_adrc = adrc;

end

