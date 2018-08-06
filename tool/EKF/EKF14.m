clear all
%% read log file
logfile = 'EKF1.LOG';

fileID = fopen(logfile, 'r');
fileDir = dir(logfile);
if(fileID)
   [LogHeader.start_time,count] = fread(fileID, [1 1], 'uint32=>uint32'); 
   [LogHeader.log_perid,count] = fread(fileID, [1 1], 'uint32=>uint32'); 
   [LogHeader.element_num,count] = fread(fileID, [1 1], 'uint32=>uint32'); 
   [LogHeader.header_size,count] = fread(fileID, [1 1], 'uint32=>uint32');
   [LogHeader.filed_size,count] = fread(fileID, [1 1], 'uint32=>uint32');
   for n = 1:LogHeader.element_num
       [LogHeader.element_info(n).name,count] = fread(fileID, [1 20], 'uint8=>char'); 
       cell{1,n} = LogHeader.element_info(n).name;
       [LogHeader.element_info(n).type,count] = fread(fileID, [1 1], 'uint32=>uint32');
       linebuff.add(n) = false;
   end

   field_num = (fileDir.bytes - LogHeader.header_size)/LogHeader.filed_size;
   % TODO: load for other type
   for n = 1:field_num
        [LogField(n,:),count] = fread(fileID, [1 LogHeader.element_num], 'float=>float');
   end
end
fclose(fileID);

index = strfind(cell, 'GPS_X');
isOne = cellfun(@(x)isequal(x,1),index);
[row,gps_x_col] = find(isOne);
index = strfind(cell, 'GPS_Y');
isOne = cellfun(@(x)isequal(x,1),index);
[row,gps_y_col] = find(isOne);
index = strfind(cell, 'BARO_ALT');
isOne = cellfun(@(x)isequal(x,1),index);
[row,baro_z_col] = find(isOne);
index = strfind(cell, 'GPS_VN');
isOne = cellfun(@(x)isequal(x,1),index);
[row,gps_vx_col] = find(isOne);
index = strfind(cell, 'GPS_VE');
isOne = cellfun(@(x)isequal(x,1),index);
[row,gps_vy_col] = find(isOne);
index = strfind(cell, 'BARO_VEL');
isOne = cellfun(@(x)isequal(x,1),index);
[row,baro_vz_col] = find(isOne);
index = strfind(cell, 'ACC_FILTER_X');
isOne = cellfun(@(x)isequal(x,1),index);
[row,acc_x_col] = find(isOne);
index = strfind(cell, 'ACC_FILTER_Y');
isOne = cellfun(@(x)isequal(x,1),index);
[row,acc_y_col] = find(isOne);
index = strfind(cell, 'ACC_FILTER_Z');
isOne = cellfun(@(x)isequal(x,1),index);
[row,acc_z_col] = find(isOne);
index = strfind(cell, 'GYR_FILTER_X');
isOne = cellfun(@(x)isequal(x,1),index);
[row,gyr_x_col] = find(isOne);
index = strfind(cell, 'GYR_FILTER_Y');
isOne = cellfun(@(x)isequal(x,1),index);
[row,gyr_y_col] = find(isOne);
index = strfind(cell, 'GYR_FILTER_Z');
isOne = cellfun(@(x)isequal(x,1),index);
[row,gyr_z_col] = find(isOne);
index = strfind(cell, 'MAG_FILTER_X');
isOne = cellfun(@(x)isequal(x,1),index);
[row,mag_x_col] = find(isOne);
index = strfind(cell, 'MAG_FILTER_Y');
isOne = cellfun(@(x)isequal(x,1),index);
[row,mag_y_col] = find(isOne);
index = strfind(cell, 'MAG_FILTER_Z');
isOne = cellfun(@(x)isequal(x,1),index);
[row,mag_z_col] = find(isOne);
index = strfind(cell, 'QUATERNION_W');
isOne = cellfun(@(x)isequal(x,1),index);
[row,qw_col] = find(isOne);
index = strfind(cell, 'QUATERNION_X');
isOne = cellfun(@(x)isequal(x,1),index);
[row,qx_col] = find(isOne);
index = strfind(cell, 'QUATERNION_Y');
isOne = cellfun(@(x)isequal(x,1),index);
[row,qy_col] = find(isOne);
index = strfind(cell, 'QUATERNION_Z');
isOne = cellfun(@(x)isequal(x,1),index);
[row,qz_col] = find(isOne);
index = strfind(cell, 'ROLL');
isOne = cellfun(@(x)isequal(x,1),index);
[row,roll_col] = find(isOne);
index = strfind(cell, 'PITCH');
isOne = cellfun(@(x)isequal(x,1),index);
[row,pitch_col] = find(isOne);
index = strfind(cell, 'YAW');
isOne = cellfun(@(x)isequal(x,1),index);
[row,yaw_col] = find(isOne);

%% init parameter
N = field_num; % count of iteration
T = 0.001*double(LogHeader.log_perid); % iteration time
g = 9.8; % gravity constant
ID_X = 1;
ID_Y = 2;
ID_Z = 3;
ID_VX = 4;
ID_VY = 5;
ID_VZ = 6;
ID_Q0 = 7;
ID_Q1 = 8;
ID_Q2 = 9;
ID_Q3 = 10;
ID_GX_BIAS = 11;
ID_GY_BIAS = 12;
ID_GZ_BIAS = 13;

%% EKF filter
% estimate covariance
q_gx = .01;
q_gy = .01;
q_gz = .01;
q_ax = .1;
q_ay = .1;
q_az = .1;
q_gx_bias = .0005;
q_gy_bias = .0005;
q_gz_bias = .0005;
% observe covariance
r_x = .05;
r_y = .05;
r_z = .05;
r_vx = .3;
r_vy = .3;
r_vz = .3;
r_ax = .0013;
r_ay = .0013;
r_az = .0013;
r_mx = .0012;
r_my = .0012;
r_mz = .0012;

% init EKF
% X = [x, y, z, vx, vy, vz, q0, q1, q2, q3, gx_bias, gy_bias, gz_bias]
X = [0; 0; 0; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0];
% U = [gx, gy, gz, ax, ay, az]
U = [0; 0; 0; 0; 0; 0];
% Z = [x, y, z, vx, vy, vz, ax, ay, az, mx, my, mz]
Z = [0; 0; 0; 0; 0; 0; 0; 0; -1; 1; 0; 0];

NUM_X = length(X);
NUM_Z = length(Z);
NUM_W = 9;
F = zeros(NUM_X, NUM_X);
H = zeros(NUM_Z, NUM_X);
P = zeros(NUM_X, NUM_X);
Q = zeros(NUM_W, NUM_W);
R = zeros(NUM_Z, NUM_Z);
G = zeros(NUM_X, NUM_W);
I = eye(NUM_X, NUM_X);
% init Q
% Q(1,1) = q_gx^2;
% Q(2,2) = q_gy^2;
% Q(3,3) = q_gz^2;
% Q(4,4) = q_ax^2;
% Q(5,5) = q_ay^2;
% Q(6,6) = q_az^2;
% Q(7,7) = q_gx_bias^2;
% Q(8,8) = q_gy_bias^2;
% Q(9,9) = q_gz_bias^2;
Q(1,1) = q_gx^2;
Q(2,2) = q_gy^2;
Q(3,3) = q_gz^2;
Q(4,4) = q_ax^2;
Q(5,5) = q_ay^2;
Q(6,6) = q_az^2;
Q(7,7) = q_gx_bias^2;
Q(8,8) = q_gy_bias^2;
Q(9,9) = q_gz_bias^2;
% init R
% R(1,1) = r_x^2;
% R(2,2) = r_y^2;
% R(3,3) = r_z^2;
% R(4,4) = r_vx^2;
% R(5,5) = r_vy^2;
% R(6,6) = r_vz^2;
% R(7,7) = r_ax^2;
% R(8,8) = r_ax^2;
% R(9,9) = r_ax^2;
% R(10,10) = r_mx^2;
% R(11,11) = r_mx^2;
% R(12,12) = r_mx^2;
R(1,1) = 0.001;
R(2,2) = 0.001;
R(3,3) = 0.008;
R(4,4) = 0.004;
R(5,5) = 0.004;
R(6,6) = 0.008;
R(7,7) = r_ax^2;
R(8,8) = r_ay^2;
R(9,9) = r_az^2;
R(10,10) = r_mx^2;
R(11,11) = r_my^2;
R(12,12) = r_mz^2;
% init P = Q
P(1,1) = 25;
P(2,2) = 25;
P(3,3) = 25;
P(4,4) = 5;
P(5,5) = 5;
P(6,6) = 5;
P(7,7) = 1e-5;
P(8,8) = 1e-5;
P(9,9) = 1e-5;
P(10,10) = 1e-5;
P(11,11) = 1e-9;
P(12,12) = 1e-9;
P(13,13) = 1e-9;
% P = eye(NUM_X, NUM_X);
% init state
X(ID_Q0) = LogField(1,qw_col);
X(ID_Q1) = LogField(1,qx_col);
X(ID_Q2) = LogField(1,qy_col);
X(ID_Q3) = LogField(1,qz_col);
X(ID_X) = LogField(1,gps_x_col);
X(ID_Y) = LogField(1,gps_y_col);
X(ID_Z) = LogField(1,baro_z_col);
X(ID_VX) = LogField(1,gps_vx_col);
X(ID_VY) = LogField(1,gps_vy_col);
X(ID_VZ) = LogField(1,baro_vz_col);
% bias
% gyr_bias = [0; 0; 0];

for i = 1 : N
    U(1) = LogField(i,gyr_x_col);
    U(2) = LogField(i,gyr_y_col);
    U(3) = LogField(i,gyr_z_col);
    U(4) = LogField(i,acc_x_col);
    U(5) = LogField(i,acc_y_col);
    U(6) = LogField(i,acc_z_col);
    
    q0 = X(ID_Q0);
    q1 = X(ID_Q1);
    q2 = X(ID_Q2);
    q3 = X(ID_Q3);
    gx = U(1) - X(ID_GX_BIAS);
    gy = U(2) - X(ID_GY_BIAS);
    gz = U(3) - X(ID_GZ_BIAS);
    ax = U(4);
    ay = U(5);
    az = U(6);
    mx = LogField(i,mag_x_col);
    my = LogField(i,mag_y_col);
    mz = LogField(i,mag_z_col);

    %% prediction step
    % calculate jocobbians of f(x,u)
	F(1,4) = 1;
	F(2,5) = 1;
	F(3,6) = 1;
	F(4,7) = 2 * (q0 * ax - q3 * ay + q2 * az);
    F(4,8) = 2 * (q1 * ax + q2 * ay + q3 * az);
    F(4,9) = 2 * (-q2 * ax + q1 * ay + q0 * az);
    F(4,10) = 2 * (-q3 * ax - q0 * ay + q1 * az);
    F(5,7) = 2 * (q3 * ax + q0 * ay - q1 * az);
    F(5,8) = 2 * (q2 * ax - q1 * ay - q0 * az);
    F(5,9) = 2 * (q1 * ax + q2 * ay + q3 * az);
    F(5,10) = 2 * (q0 * ax - q3 * ay + q2 * az);
    F(6,7) = 2 * (-q2 * ax + q1 * ay + q0 * az);
    F(6,8) = 2 * (q3 * ax + q0 * ay - q1 * az);
    F(6,9) = 2 * (-q0 * ax + q3 * ay - q2 * az);
    F(6,10) = 2 * (q1 * ax + q2 * ay + q3 * az);
	F(7,7)  = 0;
    F(7,8)  = -gx / 2;
    F(7,9)  = -gy / 2;
    F(7,10)  = -gz / 2;
    F(8,7)  = gx / 2;
    F(8,8)  = 0;
    F(8,9)  = gz / 2;
    F(8,10)  = -gy / 2;
    F(9,7)  = gy / 2;
    F(9,8)  = -gz / 2;
    F(9,9)  = 0;
    F(9,10)  = gx / 2;
    F(10,7)  = gz / 2;
    F(10,8)  = gy / 2;
    F(10,9)  = -gx / 2;
    F(10,10)  = 0;
	% dqdot/dgyr_bias
    F(7,11) = q1 / 2;
    F(7,12) = q2 / 2;
    F(7,13) = q3 / 2;
    F(8,11) = -q0 / 2;
    F(8,12) = q3 / 2;
    F(8,13) = -q2 / 2;
    F(9,11) = -q3 / 2;
    F(9,12) = -q0 / 2;
    F(9,13) = q1 / 2;
    F(10,11) = q2 / 2;
    F(10,12) = -q1 / 2;
    F(10,13) = -q0 / 2;
	
	% dVdot/dna
    G(4,4)  = -q0 * q0 - q1 * q1 + q2 * q2 + q3 * q3;
    G(4,5)  = 2 * (-q1 * q2 + q0 * q3);
    G(4,6)  = -2 * (q1 * q3 + q0 * q2);
    G(5,4)  = -2 * (q1 * q2 + q0 * q3);
    G(5,5)  = -q0 * q0 + q1 * q1 - q2 * q2 + q3 * q3;
    G(5,6)  = 2 * (-q2 * q3 + q0 * q1);
    G(6,5)  = 2 * (-q1 * q3 + q0 * q2);
    G(6,6)  = -2 * (q2 * q3 + q0 * q1);
    G(6,7)  = -q0 * q0 + q1 * q1 + q2 * q2 - q3 * q3;
    % dqdot/dnw
    G(7,1)  = q1 / 2;
    G(7,2)  = q2 / 2;
    G(7,3)  = q3 / 2;
    G(8,1)  = -q0 / 2;
    G(8,2)  = q3 / 2;
    G(8,3)  = -q2 / 2;
    G(9,1)  = -q3 / 2;
    G(9,2)  = -q0 / 2;
    G(9,3)  = q1 / 2;
    G(10,1)  = q2 / 2;
    G(10,2)  = -q1 / 2;
    G(10,3)  = -q0 / 2;
	
	G(11,7) = 1;
	G(12,8) = 1;
	G(13,9) = 1;
    
%     % 4th Runge-Kutta method
%     Xpre = X;
%     
%     q0 = X(ID_Q0);
%     q1 = X(ID_Q1);
%     q2 = X(ID_Q2);
%     q3 = X(ID_Q3);
%     gx = U(1) - X(ID_GX_BIAS);
%     gy = U(2) - X(ID_GY_BIAS);
%     gz = U(3) - X(ID_GZ_BIAS);
%     ax = U(4);
%     ay = U(5);
%     az = U(6);
%     K1 = [X(ID_VX); X(ID_VY); X(ID_VZ);...
%         (q0^2+q1^2-q2^2-q3^2)*ax+2*(q1*q2-q0*q3)*ay+2*(q1*q3+q0*q2)*az;...
%         2*(q1*q2+q0*q3)*ax+(q0^2-q1^2+q2^2-q3^2)*ay+2*(q2*q3-q0*q1)*az;...
%         2*(q1*q3-q0*q2)*ax+2*(q2*q3+q0*q1)*ay+(q0^2-q1^2-q2^2+q3^2)*az + g;...
%         (-q1*gx-q2*gy-q3*gz)/2; (q0*gx-q3*gy+q2*gz)/2; (q3*gx+q0*gy-q1*gz)/2; (-q2*gx+q1*gy+q0*gz)/2;...
% 		0; 0; 0];
%     X = Xpre + T/2*K1;
%     
%     q0 = X(ID_Q0);
%     q1 = X(ID_Q1);
%     q2 = X(ID_Q2);
%     q3 = X(ID_Q3);
%     gx = U(1) - X(ID_GX_BIAS);
%     gy = U(2) - X(ID_GY_BIAS);
%     gz = U(3) - X(ID_GZ_BIAS);
%     ax = U(4);
%     ay = U(5);
%     az = U(6);
%     K2 = [X(ID_VX); X(ID_VY); X(ID_VZ);...
%         (q0^2+q1^2-q2^2-q3^2)*ax+2*(q1*q2-q0*q3)*ay+2*(q1*q3+q0*q2)*az;...
%         2*(q1*q2+q0*q3)*ax+(q0^2-q1^2+q2^2-q3^2)*ay+2*(q2*q3-q0*q1)*az;...
%         2*(q1*q3-q0*q2)*ax+2*(q2*q3+q0*q1)*ay+(q0^2-q1^2-q2^2+q3^2)*az + g;...
%         (-q1*gx-q2*gy-q3*gz)/2; (q0*gx-q3*gy+q2*gz)/2; (q3*gx+q0*gy-q1*gz)/2; (-q2*gx+q1*gy+q0*gz)/2;...
% 		0; 0; 0];
%     X = Xpre + T/2*K2;
%     
%     q0 = X(ID_Q0);
%     q1 = X(ID_Q1);
%     q2 = X(ID_Q2);
%     q3 = X(ID_Q3);
%     gx = U(1) - X(ID_GX_BIAS);
%     gy = U(2) - X(ID_GY_BIAS);
%     gz = U(3) - X(ID_GZ_BIAS);
%     ax = U(4);
%     ay = U(5);
%     az = U(6);
%     K3 = [X(ID_VX); X(ID_VY); X(ID_VZ);...
%         (q0^2+q1^2-q2^2-q3^2)*ax+2*(q1*q2-q0*q3)*ay+2*(q1*q3+q0*q2)*az;...
%         2*(q1*q2+q0*q3)*ax+(q0^2-q1^2+q2^2-q3^2)*ay+2*(q2*q3-q0*q1)*az;...
%         2*(q1*q3-q0*q2)*ax+2*(q2*q3+q0*q1)*ay+(q0^2-q1^2-q2^2+q3^2)*az + g;...
%         (-q1*gx-q2*gy-q3*gz)/2; (q0*gx-q3*gy+q2*gz)/2; (q3*gx+q0*gy-q1*gz)/2; (-q2*gx+q1*gy+q0*gz)/2;...
% 		0; 0; 0];
%     X = Xpre + T*K3;
%     
%     q0 = X(ID_Q0);
%     q1 = X(ID_Q1);
%     q2 = X(ID_Q2);
%     q3 = X(ID_Q3);
%     gx = U(1) - X(ID_GX_BIAS);
%     gy = U(2) - X(ID_GY_BIAS);
%     gz = U(3) - X(ID_GZ_BIAS);
%     ax = U(4);
%     ay = U(5);
%     az = U(6);
%     K4 = [X(ID_VX); X(ID_VY); X(ID_VZ);...
%         (q0^2+q1^2-q2^2-q3^2)*ax+2*(q1*q2-q0*q3)*ay+2*(q1*q3+q0*q2)*az;...
%         2*(q1*q2+q0*q3)*ax+(q0^2-q1^2+q2^2-q3^2)*ay+2*(q2*q3-q0*q1)*az;...
%         2*(q1*q3-q0*q2)*ax+2*(q2*q3+q0*q1)*ay+(q0^2-q1^2-q2^2+q3^2)*az + g;...
%         (-q1*gx-q2*gy-q3*gz)/2; (q0*gx-q3*gy+q2*gz)/2; (q3*gx+q0*gy-q1*gz)/2; (-q2*gx+q1*gy+q0*gz)/2;...
% 		0; 0; 0];
%     X  = Xpre + T*(K1+2*K2+2*K3+K4)/6;
    
    % X(k|k-1) = f(X(k-1|k-1), U(k-1))
    dotX = [X(ID_VX); X(ID_VY); X(ID_VZ);...
        (q0^2+q1^2-q2^2-q3^2)*ax+2*(q1*q2-q0*q3)*ay+2*(q1*q3+q0*q2)*az;...
        2*(q1*q2+q0*q3)*ax+(q0^2-q1^2+q2^2-q3^2)*ay+2*(q2*q3-q0*q1)*az;...
        2*(q1*q3-q0*q2)*ax+2*(q2*q3+q0*q1)*ay+(q0^2-q1^2-q2^2+q3^2)*az + g;...
        (-q1*gx-q2*gy-q3*gz)/2; (q0*gx-q3*gy+q2*gz)/2; (q3*gx+q0*gy-q1*gz)/2; (-q2*gx+q1*gy+q0*gz)/2;...
		0; 0; 0];
    X = X + T * dotX;
    
	q_norm = sqrt(X(ID_Q0)^2+X(ID_Q1)^2+X(ID_Q2)^2+X(ID_Q3)^2);
	X(ID_Q0:ID_Q3) = X(ID_Q0:ID_Q3)/q_norm;
	
	% P(k|k-1) = F(k)*P(k-1|k-1)*F(k)' + Q(k)
	P = (I+F*T)*P*(I+F*T)' + T^2*G*Q*G';
	
	%% update step
	q0 = X(ID_Q0);
    q1 = X(ID_Q1);
    q2 = X(ID_Q2);
    q3 = X(ID_Q3);
    % normalize acc and mag
    acc_norm = sqrt(ax^2+ay^2+az^2);
    ax = ax/acc_norm;
    ay = ay/acc_norm;
    az = az/acc_norm;
    mag_norm = sqrt(mx^2+my^2+mz^2);
    mx = mx/mag_norm;
    my = my/mag_norm;
    mz = mz/mag_norm;
	% calculate jocobbians of h(x)
	H(1,1) = 1;
	H(2,2) = 1;
	H(3,3) = 1;
	H(4,4) = 1;
	H(5,5) = 1;
	H(6,6) = 1;
    H(7,7) = 2 * (q0 * ax - q3 * ay + q2 * az);
    H(7,8) = 2 * (q1 * ax + q2 * ay + q3 * az);
    H(7,9) = 2 * (-q2 * ax + q1 * ay + q0 * az);
    H(7,10) = 2 * (-q3 * ax - q0 * ay + q1 * az);
    H(8,7) = 2 * (q3 * ax + q0 * ay - q1 * az);
    H(8,8) = 2 * (q2 * ax - q1 * ay - q0 * az);
    H(8,9) = 2 * (q1 * ax + q2 * ay + q3 * az);
    H(8,10) = 2 * (q0 * ax - q3 * ay + q2 * az);
    H(9,7) = 2 * (-q2 * ax + q1 * ay + q0 * az);
    H(9,8) = 2 * (q3 * ax + q0 * ay - q1 * az);
    H(9,9) = 2 * (-q0 * ax + q3 * ay - q2 * az);
    H(9,10) = 2 * (q1 * ax + q2 * ay + q3 * az);
    H(10,7) = 2 * (q0 * mx - q3 * my + q2 * mz);
    H(10,8) = 2 * (q1 * mx + q2 * my + q3 * mz);
    H(10,9) = 2 * (-q2 * mx + q1 * my + q0 * mz);
    H(10,10) = 2 * (-q3 * mx - q0 * my + q1 * mz);
    H(11,7) = 2 * (q3 * mx + q0 * my - q1 * mz);
    H(11,8) = 2 * (q2 * mx - q1 * my - q0 * mz);
    H(11,9) = 2 * (q1 * mx + q2 * my + q3 * mz);
    H(11,10) = 2 * (q0 * mx - q3 * my + q2 * mz);
%     H(12,7) = 2 * (-q2 * mx + q1 * my + q0 * mz);
%     H(12,8) = 2 * (q3 * mx + q0 * my - q1 * mz);
%     H(12,9) = 2 * (-q0 * mx + q3 * my - q2 * mz);
%     H(12,10) = 2 * (q1 * mx + q2 * my + q3 * mz);

    
    accN = [(q0^2+q1^2-q2^2-q3^2)*ax+2*(q1*q2-q0*q3)*ay+2*(q1*q3+q0*q2)*az;...
            2*(q1*q2+q0*q3)*ax+(q0^2-q1^2+q2^2-q3^2)*ay+2*(q2*q3-q0*q1)*az;...
            2*(q1*q3-q0*q2)*ax+2*(q2*q3+q0*q1)*ay+(q0^2-q1^2-q2^2+q3^2)*az];
    accN = accN/norm(accN);
    magN = [(q0^2+q1^2-q2^2-q3^2)*mx+2*(q1*q2-q0*q3)*my+2*(q1*q3+q0*q2)*mz;...
            2*(q1*q2+q0*q3)*mx+(q0^2-q1^2+q2^2-q3^2)*my+2*(q2*q3-q0*q1)*mz;...
%             2*(q1*q3-q0*q2)*mx+2*(q2*q3+q0*q1)*my+(q0^2-q1^2-q2^2+q3^2)*mz];
            0];
    magN = magN/norm(magN);
	
	% Y(k) = Z(k) - h(X(k|k-1))
	y = [X(ID_X); X(ID_Y); X(ID_Z); X(ID_VX); X(ID_VY); X(ID_VZ);...
		accN(1); accN(2); accN(3); magN(1); magN(2); magN(3)];
    
    accConst = [0; 0; -1];
    magConst = [1; 0; 0];
	Z(1) = LogField(i,gps_x_col);
    Z(2) = LogField(i,gps_y_col);
    Z(3) = LogField(i,baro_z_col);
    Z(4) = LogField(i,gps_vx_col);
    Z(5) = LogField(i,gps_vy_col);
    Z(6) = LogField(i,baro_vz_col);
    Z(7) = accConst(1);
    Z(8) = accConst(2);
    Z(9) = accConst(3);
    Z(10) = magConst(1);
    Z(11) = magConst(2);
    Z(12) = magConst(3);
    
    Y = Z - y;

	% S(k) = H(k)*P(k|k-1)*H(k)' + R(k)
	S = H*P*H' + R;
	% K(k) = P(k|k-1)*H(k)'*S(k)^-1
	K = P*H'/S; 
	% X(k|k) = X(k|k-1) + K(k)*Y(k)
	X = X + K*Y;
	q_norm = sqrt(X(ID_Q0)^2+X(ID_Q1)^2+X(ID_Q2)^2+X(ID_Q3)^2);
	X(ID_Q0:ID_Q3) = X(ID_Q0:ID_Q3)/q_norm;
	% P(k|k) = P(k|k-1) - K(k)*H(k)*P(k|k-1);
	P = P - K*H*P;
    
    %% save result
    res_x(:,i) = X; 
    [roll(i), pitch(i), yaw(i)] = quad2euler(X(ID_Q0), X(ID_Q1), X(ID_Q2), X(ID_Q3), 'deg');
    
    bias(:,i) = X(ID_GX_BIAS:ID_GZ_BIAS);
end

%% plot result
index = 1:N;

figure
plot(index, LogField(:,roll_col), index, roll);
legend('Roll', 'EKF.Roll')
figure
plot(index, LogField(:,pitch_col), index, pitch);
legend('Pitch', 'EKF.Pitch')
figure
plot(index, LogField(:,yaw_col), index, yaw);
legend('Yaw', 'EKF.Yaw')
% figure
% plot(index, LogField(:,qw_col), index, res_x(ID_Q0,:));
% legend('Q0', 'EKF.Q0')
% figure
% plot(index, LogField(:,qx_col), index, res_x(ID_Q1,:));
% legend('Q1', 'EKF.Q1')
% figure
% plot(index, LogField(:,qy_col), index, res_x(ID_Q2,:));
% legend('Q2', 'EKF.Q2')
% figure
% plot(index, LogField(:,qz_col), index, res_x(ID_Q3,:));
% legend('Q3', 'EKF.Q3')
% figure
% plot(index, LogField(:,gps_x_col), index, res_x(ID_X,:));
% legend('X', 'EKF.X')
% figure
% plot(index, LogField(:,gps_y_col), index, res_x(ID_Y,:));
% legend('Y', 'EKF.Y')
% figure
% plot(index, LogField(:,baro_z_col), index, res_x(ID_Z,:));
% legend('Z', 'EKF.Z')
% figure
% plot(index, LogField(:,gps_vx_col), index, res_x(ID_VX,:));
% legend('VX', 'EKF.VX')
% figure
% plot(index, LogField(:,gps_vy_col), index, res_x(ID_VY,:));
% legend('VY', 'EKF.VY')
% figure
% plot(index, LogField(:,baro_vz_col), index, res_x(ID_VZ,:));
% legend('VZ', 'EKF.VZ')
