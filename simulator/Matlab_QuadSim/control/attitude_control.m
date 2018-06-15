euler_sp = Var.sp_att;
if Var.adrc_enable.Value == 1
    % TD Control generates setpoint rate
    rad_err = euler_sp - Sensor.att;
    roll_td_control = td_controller(roll_td_control, rad_err(1));
    pitch_td_control = td_controller(pitch_td_control, rad_err(2));

    err_rate(1) = roll_td_control.v2 - Sensor.gyr(1);
    err_rate(2) = pitch_td_control.v2 - Sensor.gyr(2);
end

euler_err = rad2deg(euler_sp - Sensor.att); % unit: deg
QuadrotorControl.u = att_controller_output(euler_err, Sensor.gyr, Var.h);

if Var.adrc_enable.Value == 1
    % TD extracts the derivative of error
    roll_td = adrc_td(roll_td, err_rate(1));
    pitch_td = adrc_td(pitch_td, err_rate(2));
    % NLSEF Control
    if comp_factor < 1
        Var.int(1) = Var.int(1) + 0.5*Var.h*err_rate(1);
        Var.int(2) = Var.int(2) + 0.5*Var.h*err_rate(2);
    else
        Var.int(1) = 0;
        Var.int(2) = 0;
    end
    QuadrotorControl.u(1) = nlsef(err_rate(1), roll_td.v2, 0.02, 120, 30*Var.h, roll_adrc.b0) + Var.int(1);
    QuadrotorControl.u(2) = nlsef(err_rate(2), pitch_td.v2, 0.02, 120, 30*Var.h, pitch_adrc.b0) + Var.int(2);
end