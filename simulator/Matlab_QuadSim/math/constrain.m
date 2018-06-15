function [ out ] = constrain( val, min_val, max_val )
%UNTITLED10 此处显示有关此函数的摘要
%   此处显示详细说明

    val(val<min_val) = min_val;
    val(val>max_val) = max_val;

    out = val;
end

