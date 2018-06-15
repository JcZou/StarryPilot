function [ new_state ] = update_quadmodel( h, th, filter, model, state )
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
    
    global Var;
    
    disturb_val = get(Var.disturb_hd, 'value');
    
    % square angular velocity of motor
    w = model.cR*th + model.b;
    % first order delay
    [ Var.lpf_state, w ] = rotor_dynamic( filter, Var.lpf_state, w );
    w(w<0) = 0;
    % deadzone of throttle
    dz = th >= model.dth;
    w = w.*dz;
    w2 = w.*w;
    % thrust = cT * w^2
    thrust = model.cT * w2;

    % calculate torque in body frame
    TB = get_torque(thrust, model);
    % calculate gyroscopic moments
    Mgyr = get_gyroscopic_moment( model.Im, state.omegaB, w );
    % total moment in body frame
    MB = TB + Mgyr;
    % add disturbance
    w = [0; 0; 0];
    if disturb_val
%         w = [sin(Var.sim_time/Var.h*pi/100)*20; 0; 0];
%         w = [50; 0; 0];
    end
    % calculate angular acceleration
    state.omega_dotB = w + model.I \ ( MB - cross(state.omegaB, model.I*state.omegaB) );
    % total disturbance
    state.total_disturb = state.omega_dotB - model.I \ TB;
    % update angular velocity
    state.omegaB = state.omegaB + state.omega_dotB * h;
    % update attitude
    state.att.q = update_quaternion( state.omegaB, state.att.q, h );
    state.att.euler = quaternion2euler( state.att.q );
    
    % total thrust in body frame
    Ftt = [0; 0; -sum(thrust)];
    % calculate rotation matrx
    R_b2i = rotation_mat_body2inertia(state.att.q);
    R_i2b = rotation_mat_inertia2body(state.att.q);
    % calculate linear velocity in body frame
    Vb = R_i2b * state.velI;
    % add disturbance
    ws = [0; 0; 0];
    if disturb_val
        ws = [0; 0; 10];
    end
    % calculate acceleration in inertia frame
    state.accI = ws + Var.g + R_b2i*( Ftt/model.mass - cross(state.omegaB, Vb) );
    state.pos_disturbance = state.accI-R_b2i*Ftt/model.mass-Var.g;
    % update linear velocity
    state.velI = state.velI + h * state.accI;
    
    % constrain the position
    fix_pos = get(Var.fix_pos_hd, 'value');
    frame_size = 0.35;
    if state.posI(1)<Var.axis(1)+frame_size | state.posI(1)>Var.axis(2)-frame_size | fix_pos
        state.velI(1) = 0;
        state.accI(1) = 0;
        state.posI(1) = constrain(state.posI(1), Var.axis(1)+frame_size, Var.axis(2)-frame_size);
    end
    if state.posI(2)<Var.axis(3)+frame_size | state.posI(2)>Var.axis(4)-frame_size | fix_pos
        state.velI(2) = 0;
        state.accI(2) = 0;
        state.posI(2) = constrain(state.posI(2), Var.axis(3)+frame_size, Var.axis(4)-frame_size);
    end
    if state.posI(3)<Var.axis(5) | state.posI(3)>Var.axis(6) | fix_pos
        state.velI(3) = 0;
        state.accI(3) = 0;
        state.posI(3) = constrain(state.posI(3), Var.axis(5), Var.axis(6));
    end
    
    % update position
    state.posI = state.posI + h * state.velI;
    
    new_state = state;
end

