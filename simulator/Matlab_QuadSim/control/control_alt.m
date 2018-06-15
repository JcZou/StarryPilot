function [ u ] = control_alt( err, last_err, h )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

global Var;

kp = 0.5;
ki = 0.2;
kd = 0.1;

Var.altInt = Var.altInt + err*ki*h;
Var.altInt = constrain(Var.altInt, -0.3, 0.3);
u = err*kp + Var.altInt + (err-last_err)/h*kd;
% add hover throttle
u = u + 0.5;

u = constrain(u, 0.01, 1.0);

end

