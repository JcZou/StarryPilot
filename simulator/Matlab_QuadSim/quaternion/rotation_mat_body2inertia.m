function [ R ] = rotation_mat_body2inertia( q )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    
    x22 = 2*q(2)*q(2);
    y22 = 2*q(3)*q(3);
    z22 = 2*q(4)*q(4);
    wx = q(1)*q(2);
    wy = q(1)*q(3);
    wz = q(1)*q(4);
    xy = q(2)*q(3);
    xz = q(2)*q(4);
    yz = q(3)*q(4);
    
    R = [ 1-y22-z22, 2*(xy-wz), 2*(xz+wy);...
          2*(xy+wz), 1-x22-z22, 2*(yz-wx);...
          2*(xz-wy), 2*(yz+wx), 1-x22-y22];
end

