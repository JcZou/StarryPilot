function sensor = get_sensor(state)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
global Var;


% Var.rb1 = rb_push(Var.rb1, state.att.euler);
% add noise
% sensor.gyr = state.omegaB + (2*rand(1)-1)*0.15;
% sensor.att = state.att.euler + (2*rand(1)-1)*0.04;
sensor.gyr = state.omegaB;
% sensor.gyr = awgn(state.omegaB,20);
sensor.att = state.att.euler;
% [Var.rb1, sensor.att] = rb_pop(Var.rb1);

end

