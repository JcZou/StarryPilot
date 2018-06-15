global Var;

if Var.sim_time - Var.adrc_plot_time >= 0.01
    Var.adrc_plot_time = Var.sim_time;
    % draw TD
    if Var.att_td.Value == 1
        addpoints(hv,Var.sim_time,Var.sp_att(1));
        addpoints(hv1,Var.sim_time,roll_td_control.v1);
        addpoints(hrad,Var.sim_time,QuadrotorState.att.euler(1));
        addpoints(hv2,Var.sim_time,roll_td_control.v2);
        addpoints(hvel,Var.sim_time,QuadrotorState.omegaB(1));
    end
    
    if Var.alt_td.Value == 1
        addpoints(gv,Var.sim_time,Var.sp_alt);
        addpoints(gv1,Var.sim_time,alt_td.v1);
        addpoints(galt,Var.sim_time,QuadrotorState.posI(3));
        addpoints(gv2,Var.sim_time,alt_td.v2);
        addpoints(gvel,Var.sim_time,QuadrotorState.velI(3));
    end

    if Var.adrc_order == 3
        % draw ESO
        addpoints(hz1,Var.sim_time,roll_adrc.z1);
    %     addpoints(hs1,Var.sim_time,roll_adrc.s1);
        addpoints(hx1,Var.sim_time,QuadrotorState.att.euler(1));
        addpoints(hz2,Var.sim_time,roll_adrc.z2);
    %     addpoints(hs2,Var.sim_time,roll_adrc.s2);
        addpoints(hx2,Var.sim_time,QuadrotorState.omegaB(1));
        if Var.adrc_cleso == 1
            addpoints(hz3,Var.sim_time,(roll_adrc.z3+roll_adrc.s3));
        else
            addpoints(hz3,Var.sim_time,(roll_adrc.z3));
        end
    %     addpoints(hs3,Var.sim_time,roll_adrc.s3);
        addpoints(hx3,Var.sim_time,QuadrotorState.total_disturb(1));
    elseif Var.adrc_order == 2
        % draw ESO
        if Var.att_eso.Value == 1
            addpoints(hz1,Var.sim_time,roll_adrc.z1);
            addpoints(hx1,Var.sim_time,QuadrotorState.omegaB(1));
            if Var.adrc_cleso == 1
                addpoints(hz2,Var.sim_time,(roll_adrc.z2+roll_adrc.s2));
            else
                addpoints(hz2,Var.sim_time,(roll_adrc.z2));
            end
            addpoints(hx2,Var.sim_time,QuadrotorState.total_disturb(1));
            addpoints(hx3,Var.sim_time,QuadrotorState.att.euler(1));
        end
        
        if Var.alt_eso.Value == 1
            addpoints(gz1,Var.sim_time,alt_adrc.z1);
            addpoints(gx1,Var.sim_time,QuadrotorState.velI(3));
            addpoints(gz2,Var.sim_time,(alt_adrc.z2));
            addpoints(gx2,Var.sim_time,QuadrotorState.pos_disturbance(3));
            addpoints(gx3,Var.sim_time,QuadrotorState.posI(3));
            addpoints(gref,Var.sim_time, Var.sp_alt);
        end
    end
end