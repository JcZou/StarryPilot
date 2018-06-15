function [ y ] = fal( x, alpha, delta )
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明

if abs(x) < delta
    y = x / delta^(1-alpha);
else
    y = (abs(x)^alpha) * sign(x);
end

end

