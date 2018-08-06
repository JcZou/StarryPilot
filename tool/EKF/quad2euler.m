function [roll, pitch, yaw] = quad2euler(q0, q1, q2, q3, unit)

	ysqr = q2 * q2;

	% roll (x-axis rotation)
	t0 = 2 * (q0 * q1 + q2 * q3);
	t1 = 1 - 2 * (q1 * q1 + ysqr);
	roll = atan2(t0, t1);

	% pitch (y-axis rotation)
	t2 = 2 * (q0 * q2 - q3 * q1);
    t2 = trinocular_op(t2 > 1,1,t2);
    t2 = trinocular_op(t2 < -1,-1,t2);
	pitch = asin(t2);

	% yaw (z-axis rotation)
	t3 = 2 * (q0 * q3 + q1 *q2);
	t4 = 1 - 2 * (ysqr + q3 * q3);  
	yaw = atan2(t3, t4);

    if unit == 'deg'
       roll = roll/pi*180; 
       pitch = pitch/pi*180;
       yaw = yaw/pi*180;
    end
end

