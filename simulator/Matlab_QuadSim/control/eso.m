if Var.adrc_order == 3
    roll_adrc = adrc_cleso(roll_adrc, QuadrotorControl.u(1), QuadrotorState.att.euler(1), Var.adrc_order);
    pitch_adrc = adrc_cleso(pitch_adrc, QuadrotorControl.u(2), QuadrotorState.att.euler(2), Var.adrc_order);
elseif Var.adrc_order == 2
    roll_adrc = adrc_cleso(roll_adrc, QuadrotorControl.u(1), Sensor.gyr(1), Var.adrc_order);
    pitch_adrc = adrc_cleso(pitch_adrc, QuadrotorControl.u(2), Sensor.gyr(2), Var.adrc_order);

%         alt_adrc = adrc_cleso(alt_adrc, -u2+9.8/alt_adrc.b0, QuadrotorState.velI(3), Var.adrc_order);
    alt_adrc = adrc_cleso(alt_adrc, -base_throttle*base_throttle+9.8/alt_adrc.b0, QuadrotorState.velI(3), Var.adrc_order);
end