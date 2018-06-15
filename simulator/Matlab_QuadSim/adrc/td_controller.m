function new = td_controller(td_control, e)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
td_control.v1 = td_control.v1 + td_control.h*td_control.v2;
td_control.v2 = td_control.v2 + td_control.h * fhan(-e, td_control.v2, td_control.r1, td_control.h1);
new = td_control;

end

