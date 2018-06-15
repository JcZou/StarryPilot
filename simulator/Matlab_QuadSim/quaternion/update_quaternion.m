function [ new_q ] = update_quaternion( w, q, h )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

    W = [0 -w(1) -w(2) -w(3); w(1) 0 w(3) -w(2); w(2) -w(3) 0 w(1); w(3) w(2) -w(1) 0];
    q_dot = 0.5 * W * q;
    new_q = q + q_dot * h;
end

