function [ m_gyro ] = get_gyroscopic_moment( Im, w, mw )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    ez = [0; 0; 1];
    wt = -mw(1) - mw(2) + mw(3) + mw(4);
    
    m_gyro = cross(Im * ez, w * wt);
end

