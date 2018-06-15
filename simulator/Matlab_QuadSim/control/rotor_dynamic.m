function [ new_x, new_y ] = rotor_dynamic( sysd, x, u )
%UNTITLED11 此处显示有关此函数的摘要
%   此处显示详细说明

    new_x = sysd.A*x + sysd.B*u;
    new_y = sysd.C*x + sysd.D*u;
end

