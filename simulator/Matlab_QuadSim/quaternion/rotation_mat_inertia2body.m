function [ R ] = rotation_mat_inertia2body( q )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

    r = rotation_mat_body2inertia( q );
    R = r';
end

