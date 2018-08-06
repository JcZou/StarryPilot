function [res] = trinocular_op(cond,v1,v2)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
if cond
    res = v1;
else
    res = v2;
end
end

