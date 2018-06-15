%clearvars;

% parameters of drone model
DroneModel.Km = 1;  % amplified factor
DroneModel.T = quadModel.T; % time constant of motor
DroneModel.kt = quadModel.cr;
DroneModel.bt = quadModel.b;
DroneModel.L = quadModel.arms_L;   % arm length: meter
DroneModel.cT = quadModel.ct*10000; % the unit of throtle in document is 0~100, we need change to 0~1
DroneModel.cQ = quadModel.cq*10000;
DroneModel.Im = quadModel.Jm;  % motor inertia
DroneModel.mass = quadModel.mass; % total mass: kg
DroneModel.Ixx = quadModel.Jx;
DroneModel.Iyy = quadModel.Jy;
DroneModel.Izz = quadModel.Jz;
DroneModel.I = [DroneModel.Ixx 0 0; 0 DroneModel.Iyy 0; 0 0 DroneModel.Izz];
DroneModel.airframe = 'X';  % X configuration

% init drone states
DroneState.att.euler = [0 0 0]'; % rad
DroneState.omegaB = [0 0 0]'; % rad/s
DroneState.omega_dotB = [0 0 0]'; % rad/s/s
DroneState.accI = [0 0 0]'; % m/s/s
DroneState.velI = [0 0 0]'; % m/s
DroneState.posI = [0 0 0]'; % m
DroneState.att.q = euler2quaternion(DroneState.att.euler);

% init control signal
ControlSignal.basethrottle = 0.5; % throttle range from 0.0 to 1.0
ControlSignal.u = [0 0 0]';
ControlSignal.throttle = [0 0 0 0]';% throttle for motor 1~4

% simulation time: s
Simu_T = 10;
% samping time: s
Samp_T = 0.002;

% motor system, first order
motor_sys = ss(tf(DroneModel.Km, [DroneModel.T, 1]));
motor_sysd = c2d(motor_sys, Samp_T);

motor_x_state = [0 0 0 0]';

% gravity constant
g = [0; 0; 9.81];

k = 0;
last_err = [0;0;0];

% simulation loop
for time = 0 : Samp_T : Simu_T
    k = k+1;
    
    % get target attitude: rad
    euler_sp = get_target_euler(time);
    % get current euler angle of drone
    euler_cur = quaternion2euler(DroneState.att.q);
    % calculate attitude error
    euler_err = euler_sp - euler_cur;
    % calculate control signal
    ControlSignal.u = att_controller_output(euler_err, last_err, Samp_T);
    last_err = euler_err;
    
    % throttle mix
    ControlSignal.throttle = throttle_mix(ControlSignal.basethrottle, ControlSignal.u, DroneModel.airframe);
    ControlSignal.throttle
    % rotor dynamics
    mw2 = DroneModel.kt * ControlSignal.throttle + DroneModel.bt;
    mw2 = max(0 , mw2);
    % inertia link to generate delay
    [ motor_x_state, mw2_delay ] = rotor_dynamic( motor_sysd, motor_x_state, mw2 );
    
    % calculate torque in body frame
    TB = get_torque(mw2_delay, DroneModel.L, DroneModel.cT, DroneModel.cQ, DroneModel.airframe);
    % calculate gyroscopic moments
    Mgyr = get_gyroscopic_moment( DroneModel.Im, DroneState.omegaB, sqrt(mw2_delay) );
    % total moment in body frame
    MB = TB + Mgyr;
    % calculate angular acceleration
    DroneState.omega_dotB = DroneModel.I \ ( MB - cross(DroneState.omegaB, DroneModel.I*DroneState.omegaB) );
    % update angular velocity
    DroneState.omegaB = DroneState.omegaB + DroneState.omega_dotB * Samp_T;
    % update attitude
    DroneState.att.q = update_quaternion( DroneState.omegaB, DroneState.att.q, Samp_T );
    DroneState.att.euler = quaternion2euler( DroneState.att.q );
    
    % total thrust in body frame
    Ftt = [0; 0; -DroneModel.cT * sum(mw2_delay)];
    % calculate rotation matrx
    R_b2i = rotation_mat_body2inertia(DroneState.att.q);
    R_i2b = rotation_mat_inertia2body(DroneState.att.q);
    % calculate linear velocity in body frame
    Vb = R_i2b * DroneState.velI;
    % calculate acceleration in inertia frame
    DroneState.accI = g + R_b2i*( Ftt/DroneModel.mass - cross(DroneState.omegaB, Vb) );
    % update linear velocity
    DroneState.velI = DroneState.velI + Samp_T * DroneState.accI;
    % update position
    DroneState.posI = DroneState.posI + Samp_T * DroneState.velI;
    
    % record value
    vik(:,k) = DroneState.velI;
    aik(:,k) = DroneState.accI;
    eulark(:,k) = DroneState.att.euler;
end

tt = 0 : Samp_T : Simu_T;
% figure
% plot(tt, vik(1,:));
% legend('vx');
% figure
% plot(tt, aik(1,:));
% legend('ax');
figure
plot(tt, eulark(1,:));
legend('euler x');
figure
plot(tt, eulark(2,:));
legend('euler y');
figure
plot(tt, eulark(3,:));
legend('euler z');
