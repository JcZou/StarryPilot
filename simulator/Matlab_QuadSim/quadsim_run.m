%	StarryPilot MatSim
%	Author: Jiachi Zou
%	06/15/2018

clear all;
clc;

global Var;

%% initialization
init_var;
init_disp;

% ADRC_ENABLE = 0;

%% execute simulator
while 1
    start_val = get(Var.start_hd, 'value');
    if ~start_val
        drawnow limitrate;
        continue;
    end
    
    k = k + 1;
    
    %% read sensor data
    Sensor = get_sensor(QuadrotorState);

    % ESO
    eso;

    %% Quadrotor Control
    % altitude control
    altitude_control;
    % attitude control
    attitude_control;
    % disturbance compensation
    disturb_comp;

    % test example
    test;    
    
    %% mix throttle according to airframe configuration
    QuadrotorControl.throttle = throttle_mix(base_throttle, QuadrotorControl.u, QuadrotorModel.airframe);
    % compensate throttle due to the delay of motor
    throttle_compensate;

    %% update quadrotor model states
    QuadrotorState = update_quadmodel( Var.h, QuadrotorControl.throttle, motor_sysd, QuadrotorModel, QuadrotorState );
    QuadrotorState.att.q = euler2quaternion(QuadrotorState.att.euler);
    %% update 3D model
    Quadrotor3D = update_quadrotor_3D( Quadrotor3D, QuadrotorState );
    draw_quadrotor_3D( Quadrotor3D, QuadrotorState );
    
    %% record and update UI
    record;
    updateUI;
    draw_adrc;
end
