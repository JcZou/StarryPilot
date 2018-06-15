alt_td = adrc_td2(alt_td, Var.sp_alt);  % unit: rad   
alt_err = -(alt_td.v1 - QuadrotorState.posI(3));

% pid
if Var.adrc_enable.Value == 0
    base_throttle = control_alt(alt_err, last_alt_err, Var.h);
    last_alt_err = alt_err;
else
    % adrc
    alt_u = nlsef(alt_err, (QuadrotorState.velI(3)-alt_td.v2), 2, alt_adrc.b0*0.5, 10*Var.h, alt_adrc.b0);
    base_throttle = 0.5 + alt_u;
    base_throttle = constrain(base_throttle, 0.01, 1);
end