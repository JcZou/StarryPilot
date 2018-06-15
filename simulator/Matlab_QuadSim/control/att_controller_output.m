function [ out ] = att_controller_output( err, gyr, h )

global Var;

roll_p = 0.1;
roll_rate_p = 0.04;
roll_rate_i = 0.05;
roll_rate_d = 0.0004;
pitch_p = 0.1;
pitch_rate_p = 0.04;
pitch_rate_i = 0.05;
pitch_rate_d = 0.0004;
yaw_p = 0.1;
yaw_rate_p = 0.2;
yaw_rate_i = 0.05;
yaw_rate_d = 0.0005;

% out control ring
rate_sp = [roll_p; pitch_p; yaw_p].*err;

% inner control ring
err_rate = rate_sp - gyr;
deriv = [roll_rate_d; pitch_rate_d; yaw_rate_d] .* (err_rate - Var.last_err_rate)/h;
Var.int = Var.int + [roll_rate_i; pitch_rate_i; yaw_rate_i] .* err_rate * h;
Var.int = constrain(Var.int, -0.2, 0.2);
out = [roll_rate_p; pitch_rate_p; yaw_rate_p].*err_rate + Var.int + deriv;
Var.last_err_rate = err_rate;

% adrc in inner loop


out = reshape(out, [3, 1]);
out = constrain(out, -0.4, 0.4);

end

