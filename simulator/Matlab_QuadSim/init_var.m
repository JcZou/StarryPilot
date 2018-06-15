%load QuadrotorModel;
load QuadrotorState;
load QuadrotorControl;
load Quadrotor3D; 

QuadrotorModel.Km = 1;
QuadrotorModel.T = 0.025;   % motor constant, s
QuadrotorModel.L = 0.255;   % arm length, m
QuadrotorModel.cR = 718.078;  % w = cR*throttle + b
% QuadrotorModel.b = 88.448;
QuadrotorModel.b = 0;
QuadrotorModel.cT = 1.23884e-5; % thrust = cT*w^2
QuadrotorModel.c1 = 6.9803;
QuadrotorModel.c2 = 1.419;
QuadrotorModel.c3 = 0.0362;
QuadrotorModel.tc = 0.016;  % torque constant
QuadrotorModel.Im = 3.789*1e-6; % inertia of rotor
QuadrotorModel.mass = 0.886;   % total mass
QuadrotorModel.dth = 0.06;  % dead throttle
QuadrotorModel.Ixx = 0.016;
QuadrotorModel.Iyy = 0.016;
QuadrotorModel.Izz = 0.0274;
QuadrotorModel.I = [QuadrotorModel.Ixx,0,0;0,QuadrotorModel.Iyy,0;0,0,QuadrotorModel.Izz];
QuadrotorModel.airframe = 'X';

global Var;

% simulation time: s
Var.sim_time = 0;
% samping time: s
Var.h = 0.004;

% motor system, first order
motor_sys = ss(tf(QuadrotorModel.Km, [QuadrotorModel.T, 1]));
motor_sysd = c2d(motor_sys, Var.h);
motor_x_state = [0 0 0 0]';

% gravity constant
Var.g = [0; 0; 9.8];

% axes range
Var.axis = [-3 3 -3 3 -3 0];

% used for 1 order filter
Var.lpf_state = [0;0;0;0];

roll_adrc.h = Var.h;
roll_adrc.z1 = 0;
roll_adrc.z2 = 0;
roll_adrc.z3 = 0;
roll_adrc.s1 = 0;
roll_adrc.s2 = 0;
roll_adrc.s3 = 0;
roll_adrc.v1 = 0;
roll_adrc.v2 = 0;
roll_adrc.r0 = 6;
roll_adrc.b0 = 350;   % will be calculated by eso
roll_adrc.beta1 = 100;
roll_adrc.beta2 = 300;
roll_adrc.beta3 = 1000;
roll_adrc.c = 0.5;
roll_adrc.r = 100;
roll_adrc.h1 = 20*roll_adrc.h;
roll_adrc.w = 120;

pitch_adrc.h = Var.h;
pitch_adrc.z1 = 0;
pitch_adrc.z2 = 0;
pitch_adrc.z3 = 0;
pitch_adrc.s1 = 0;
pitch_adrc.s2 = 0;
pitch_adrc.s3 = 0;
pitch_adrc.v1 = 0;
pitch_adrc.v2 = 0;
pitch_adrc.r0 = 6;
pitch_adrc.b0 = 350;   % will be calculated by eso
pitch_adrc.beta1 = 100;
pitch_adrc.beta2 = 300;
pitch_adrc.beta3 = 1000;
pitch_adrc.c = 0.5;
pitch_adrc.r = 100;
pitch_adrc.h1 = 20*pitch_adrc.h;
pitch_adrc.w = 120;

alt_adrc.h = Var.h;
alt_adrc.z1 = 0;
alt_adrc.z2 = 0;
alt_adrc.z3 = 0;
alt_adrc.s1 = 0;
alt_adrc.s2 = 0;
alt_adrc.s3 = 0;
alt_adrc.v1 = 0;
alt_adrc.v2 = 0;
alt_adrc.r0 = 6;
alt_adrc.b0 = (4*QuadrotorModel.cT*(QuadrotorModel.cR^2))/QuadrotorModel.mass;
alt_adrc.c = 0.5;
alt_adrc.r = 100;
alt_adrc.h1 = 10*alt_adrc.h;
alt_adrc.w = 120;

Var.altInt = 0;
Var.sp_att = [0; 0; 0];
Var.sp_alt = 0;
Var.last_err_rate = [0; 0; 0];
Var.int = [0; 0; 0];
Var.adrc_plot_time = 0;
Var.ringbuffer.head = 0;
Var.ringbuffer.tail = 0;
Var.ringbuffer.size = 3;
Var.ringbuffer.buffer(1) = 0;
Var.ringbuffer.buffer(2) = 0;
Var.ringbuffer.buffer(3) = 0;
Var.adrc_order = 2;
Var.adrc_cleso = 0;

% roll_td.v1 = 0;
% roll_td.v2 = 0;
% roll_td.r0 = 1000;
% roll_td.h = Var.h;
% pitch_td.v1 = 0;
% pitch_td.v2 = 0;
% pitch_td.r0 = 1000;
% pitch_td.h = Var.h;

Var.td1.v1 = 0;
Var.td1.v2 = 0;
Var.td1.r0 = 1e8;
Var.td1.h = Var.h;
Var.td2.v1 = 0;
Var.td2.v2 = 0;
Var.td2.r0 = 1e8;
Var.td2.h = Var.h;
Var.td3.v1 = 0;
Var.td3.v2 = 0;
Var.td3.r0 = 1e8;
Var.td3.h = Var.h;
Var.td4.v1 = 0;
Var.td4.v2 = 0;
Var.td4.r0 = 1e8;
Var.td4.h = Var.h;

roll_td_control.h = Var.h;
roll_td_control.v1 = 0;
roll_td_control.v2 = 0;
roll_td_control.r1 = 49;
roll_td_control.h1 = 10*Var.h;
pitch_td_control.h = Var.h;
pitch_td_control.v1 = 0;
pitch_td_control.v2 = 0;
pitch_td_control.r1 = 49;
pitch_td_control.h1 = 10*Var.h;

roll_td.h = Var.h;
roll_td.v1 = 0;
roll_td.v2 = 0;
roll_td.r0 = 500;
pitch_td.h = Var.h;
pitch_td.v1 = 0;
pitch_td.v2 = 0;
pitch_td.r0 = 500;

alt_td.h = Var.h;
alt_td.v1 = 0;
alt_td.v2 = 0;
alt_td.r0 = 10;
alt_td.h1 = 20*Var.h;

%% other parameters initialize
last_alt_err = 0;
base_throttle = 0;
k = 0;
Record.id = 1;
b0_const = 8*QuadrotorModel.cR*QuadrotorModel.cT*QuadrotorModel.L*sin(pi/4);
comp_factor = 1;