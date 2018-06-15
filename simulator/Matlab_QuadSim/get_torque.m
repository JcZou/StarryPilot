function [ torque ] = get_torque( T, model )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    if model.airframe == 'X'
        l = model.L * sin(pi/4);
        mat = [-l, l, l, -l; l, -l, l, -l; model.tc, model.tc, -model.tc, -model.tc];
        torque = mat * T;
    else
        disp('unknown airframe');
    end
end

