function [ e ] = quaternion2euler( q )
%UNTITLED5 此处显示有关此函数的摘要
%   此处显示详细说明

	ysqr = q(3) * q(3);

	% roll (x-axis rotation)
	t0 = 2.0 * (q(1) * q(2) + q(3) * q(4));
	t1 = 1.0 - 2.0 * (q(2) * q(2) + ysqr);
	e(1) = atan2(t0, t1);

	% pitch (y-axis rotation)
	t2 = 2.0 * (q(1) * q(3) - q(4) * q(2));
    t2 = min(1.0, t2);
    t2 = max(-1.0, t2);
	e(2) = asin(t2);

	% yaw (z-axis rotation)
	t3 = 2.0 * (q(1) * q(4) + q(2) *q(3));
	t4 = 1.0 - 2.0 * (ysqr + q(4) * q(4));  
	e(3) = atan2(t3, t4);

    e = reshape(e, [3, 1]);
end

