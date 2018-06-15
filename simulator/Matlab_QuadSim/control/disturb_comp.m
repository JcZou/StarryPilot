if Var.adrc_enable.Value == 1
    % total disturbance compensation
    if Var.adrc_order == 3
        if Var.adrc_cleso == 1
            QuadrotorControl.u(1) = QuadrotorControl.u(1) - comp_factor*(roll_adrc.z3+roll_adrc.s3)/roll_adrc.b0;
            QuadrotorControl.u(2) = QuadrotorControl.u(2) - comp_factor*(pitch_adrc.z3+pitch_adrc.s3)/pitch_adrc.b0;
        else
            QuadrotorControl.u(1) = QuadrotorControl.u(1) - comp_factor*(roll_adrc.z3)/roll_adrc.b0;
            QuadrotorControl.u(2) = QuadrotorControl.u(2) - comp_factor*(pitch_adrc.z3)/pitch_adrc.b0;
        end
    elseif Var.adrc_order == 2
        if Var.adrc_cleso == 1
            QuadrotorControl.u(1) = QuadrotorControl.u(1) - comp_factor*(roll_adrc.z2+roll_adrc.s2)/roll_adrc.b0;
            QuadrotorControl.u(2) = QuadrotorControl.u(2) - comp_factor*(pitch_adrc.z2+pitch_adrc.s2)/pitch_adrc.b0;
        else
            QuadrotorControl.u(1) = QuadrotorControl.u(1) - comp_factor*(roll_adrc.z2)/roll_adrc.b0;
            QuadrotorControl.u(2) = QuadrotorControl.u(2) - comp_factor*(pitch_adrc.z2)/pitch_adrc.b0;
            
%                 base_throttle = base_throttle^2;
                u2 = base_throttle + comp_factor*(alt_adrc.z2)/alt_adrc.b0;
                u2 = constrain(u2, 0, 1);
                base_throttle = sqrt(u2);
                base_throttle = constrain(base_throttle, 0.01, 1);
        end
    end
    QuadrotorControl.u = constrain( QuadrotorControl.u, -0.5, 0.5 );
end

roll_adrc.b0 = b0_const*(QuadrotorModel.cR*base_throttle+QuadrotorModel.b)/QuadrotorModel.Ixx;
pitch_adrc.b0 = b0_const*(QuadrotorModel.cR*base_throttle+QuadrotorModel.b)/QuadrotorModel.Iyy;