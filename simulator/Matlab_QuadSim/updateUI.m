
% update simulation time
Var.sim_time = Var.sim_time + Var.h;
set(Var.st_hd ,'string',num2str(Var.sim_time));
set(Var.x_hd ,'string',num2str(QuadrotorState.posI(1)));
set(Var.y_hd ,'string',num2str(QuadrotorState.posI(2)));
set(Var.z_hd ,'string',num2str(QuadrotorState.posI(3)));
set(Var.roll_hd ,'string',num2str(QuadrotorState.att.euler(1)));
set(Var.pitch_hd ,'string',num2str(QuadrotorState.att.euler(2)));
set(Var.yaw_hd ,'string',num2str(QuadrotorState.att.euler(3)));
set(Var.th_hd ,'string',num2str(Var.sp_alt));
set(Var.sp_roll_hd ,'string',num2str(Var.sp_att(1)));
set(Var.sp_pitch_hd ,'string',num2str(Var.sp_att(2)));
set(Var.sp_yaw_hd ,'string',num2str(Var.sp_att(3)));
set(Var.motor1_hd ,'string',num2str(QuadrotorControl.throttle(1)));
set(Var.motor2_hd ,'string',num2str(QuadrotorControl.throttle(2)));
set(Var.motor3_hd ,'string',num2str(QuadrotorControl.throttle(3)));
set(Var.motor4_hd ,'string',num2str(QuadrotorControl.throttle(4)));
drawnow limitrate;