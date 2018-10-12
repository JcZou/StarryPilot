clear all
%% read log file
logfile = 'HIL.LOG';
readlog;

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
ID_AZ_BIAS = 14;

%% EKF filter
% estimate covariance
q_gx = .0025;
q_gy = .0025;
q_gz = .0025;
q_ax = .1;
q_ay = .1;
q_az = .1;
q_gx_bias = .0005;
q_gy_bias = .0005;
q_gz_bias = .0005;
% q_gx_bias = .001;
% q_gy_bias = .001;
% q_gz_bias = .001;
q_az_bias = .0025;
% q_az_bias = .01;
% observe covariance
r_x = .02;
r_y = .02;
r_z = .04;
r_vx = .035;
r_vy = .035;
r_vz = .08;
r_ax = .001;
r_ay = .0015;
r_az = .0015;
r_mx = .0012;
r_my = .0012;
r_mz = .0012;

% init EKF
EKF_Init;

% execute EKF
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
    az = U(6) - X(ID_AZ_BIAS);
    mx = LogField(i,mag_x_col);
    my = LogField(i,mag_y_col);
    mz = LogField(i,mag_z_col);

    %% prediction step
    % calculate jocobbians of f(x,u)
    % d(POSdot)/d(V)
	F(1,4) = 1;
	F(2,5) = 1;
	F(3,6) = 1;
    % d(Vdoot)/d(q)
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
	% d(Vdot)/d(acc_bias）
    F(4,14) =  -2 * (q1 * q3 + q0 * q2);
	F(5,14) =  2 * (-q2 * q3 + q0 * q1);
	F(6,14) =  -q0 * q0 + q1 * q1 + q2 * q2 - q3 * q3;
	% d(qdot)/d(q)
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
	% d(qdot)/d(gyr_bias)
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
	
	% d(Vdot)/d(acc)
    G(4,4)  = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    G(4,5)  = 2 * (q1 * q2 - q0 * q3);
    G(4,6)  = 2 * (q1 * q3 + q0 * q2);
    G(5,4)  = 2 * (q1 * q2 + q0 * q3);
    G(5,5)  = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
    G(5,6)  = 2 * (q2 * q3 - q0 * q1);
	G(6,4)  = 2 * (q1 * q3 - q0 * q2);
    G(6,5)  = 2 * (q2 * q3 + q0 * q1);
    G(6,6)  = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    % d(qdot)/d(gyr)
    G(7,1)  = -q1 / 2;
    G(7,2)  = -q2 / 2;
    G(7,3)  = -q3 / 2;
    G(8,1)  = q0 / 2;
    G(8,2)  = -q3 / 2;
    G(8,3)  = q2 / 2;
    G(9,1)  = q3 / 2;
    G(9,2)  = q0 / 2;
    G(9,3)  = -q1 / 2;
    G(10,1)  = -q2 / 2;
    G(10,2)  = q1 / 2;
    G(10,3)  = q0 / 2;
    % d(bias)/d(bias)
	G(11,7) = 1;
	G(12,8) = 1;
	G(13,9) = 1;
    G(14,10) = 1;
    
    % X(k|k-1) = f(X(k-1|k-1), U(k-1))
    dotX = [X(ID_VX); X(ID_VY); X(ID_VZ);...
        (q0^2+q1^2-q2^2-q3^2)*ax+2*(q1*q2-q0*q3)*ay+2*(q1*q3+q0*q2)*az;...
        2*(q1*q2+q0*q3)*ax+(q0^2-q1^2+q2^2-q3^2)*ay+2*(q2*q3-q0*q1)*az;...
        2*(q1*q3-q0*q2)*ax+2*(q2*q3+q0*q1)*ay+(q0^2-q1^2-q2^2+q3^2)*az + g;...
        (-q1*gx-q2*gy-q3*gz)/2; (q0*gx-q3*gy+q2*gz)/2; (q3*gx+q0*gy-q1*gz)/2; (-q2*gx+q1*gy+q0*gz)/2;...
		0; 0; 0; 0;];
    X = X + T * dotX;
    
	q_norm = sqrt(X(ID_Q0)^2+X(ID_Q1)^2+X(ID_Q2)^2+X(ID_Q3)^2);
	X(ID_Q0:ID_Q3) = X(ID_Q0:ID_Q3)/q_norm;
	
	% P(k|k-1) = (I+F(k)*T)*P(k-1|k-1)*(I+F(k)*T)' + T^2*G*Q(k)*G'
	P = (I+F*T)*P*(I+F*T)' + T^2*G*Q*G';
    
    if ax==0 && ay==0 && az==0
       continue; 
    end
    if mx==0 && my==0 && mz==0
       continue; 
    end
    
    %% Use history observer value
    % store history data
    hist_x(:,i) = X;
    % get history data
    for n = 1:NUM_X
        HistX(n,1) = hist_x(n,trinocular_op(i>HistIndex(n), i-HistIndex(n), 1));
    end
    % calculate delta state
    DeltaX = X - HistX;
    % roll-back to history data
    X = HistX;
	
	%% update step
	q0 = X(ID_Q0);
    q1 = X(ID_Q1);
    q2 = X(ID_Q2);
    q3 = X(ID_Q3);
    ax = U(4);
    ay = U(5);
    az = U(6);
    
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
    % d(X)/d(X)
	H(1,1) = 1;
	H(2,2) = 1;
	H(3,3) = 1;
    % d(V)/d(V)
% 	H(4,4) = 1;
% 	H(5,5) = 1;
% 	H(6,6) = 1;
    % d(acc)/d(q)
    H(4,7) = 2 * (q0 * ax - q3 * ay + q2 * az);
    H(4,8) = 2 * (q1 * ax + q2 * ay + q3 * az);
    H(4,9) = 2 * (-q2 * ax + q1 * ay + q0 * az);
    H(4,10) = 2 * (-q3 * ax - q0 * ay + q1 * az);
    H(5,7) = 2 * (q3 * ax + q0 * ay - q1 * az);
    H(5,8) = 2 * (q2 * ax - q1 * ay - q0 * az);
    H(5,9) = 2 * (q1 * ax + q2 * ay + q3 * az);
    H(5,10) = 2 * (q0 * ax - q3 * ay + q2 * az);
    H(6,7) = 2 * (-q2 * ax + q1 * ay + q0 * az);
    H(6,8) = 2 * (q3 * ax + q0 * ay - q1 * az);
    H(6,9) = 2 * (-q0 * ax + q3 * ay - q2 * az);
    H(6,10) = 2 * (q1 * ax + q2 * ay + q3 * az);
    % d(mag)/d(q)
    H(7,7) = 2 * (q0 * mx - q3 * my + q2 * mz);
    H(7,8) = 2 * (q1 * mx + q2 * my + q3 * mz);
    H(7,9) = 2 * (-q2 * mx + q1 * my + q0 * mz);
    H(7,10) = 2 * (-q3 * mx - q0 * my + q1 * mz);
    H(8,7) = 2 * (q3 * mx + q0 * my - q1 * mz);
    H(8,8) = 2 * (q2 * mx - q1 * my - q0 * mz);
    H(8,9) = 2 * (q1 * mx + q2 * my + q3 * mz);
    H(8,10) = 2 * (q0 * mx - q3 * my + q2 * mz);
    

    % rotate acc and mag from body frame to navigation frame
    accN = [(q0^2+q1^2-q2^2-q3^2)*ax+2*(q1*q2-q0*q3)*ay+2*(q1*q3+q0*q2)*az;...
            2*(q1*q2+q0*q3)*ax+(q0^2-q1^2+q2^2-q3^2)*ay+2*(q2*q3-q0*q1)*az;...
            2*(q1*q3-q0*q2)*ax+2*(q2*q3+q0*q1)*ay+(q0^2-q1^2-q2^2+q3^2)*az];
    accN = accN/norm(accN);
    magN = [(q0^2+q1^2-q2^2-q3^2)*mx+2*(q1*q2-q0*q3)*my+2*(q1*q3+q0*q2)*mz;...
            2*(q1*q2+q0*q3)*mx+(q0^2-q1^2+q2^2-q3^2)*my+2*(q2*q3-q0*q1)*mz;...
            0];
    magN = magN/norm(magN);
	
	% Y(k) = Z(k) - h(X(k|k-1))
	y = [X(ID_X); X(ID_Y); X(ID_Z);...
		accN(1); accN(2); accN(3); magN(1); magN(2)];
    
    GPS_VX = 0;
    GPS_VY = 0;
    if(i>50)
        GPS_VX = (LogField(i,gps_x_col)-LogField(i-50,gps_x_col))/0.1;
        GPS_VY = (LogField(i,gps_y_col)-LogField(i-50,gps_y_col))/0.1;
        LogField(i,gps_vx_col) = GPS_VX;
        LogField(i,gps_vy_col) = GPS_VY;
    end
    
    accConst = [0; 0; -1];
    magConst = [1; 0];
	Z(1) = LogField(i,gps_x_col);
    Z(2) = LogField(i,gps_y_col);
    Z(3) = LogField(i,baro_z_col);
%     Z(4) = LogField(i,gps_vx_col);
%     Z(5) = LogField(i,gps_vy_col);
%     Z(6) = LogField(i,baro_vz_col);
    Z(4) = accConst(1);
    Z(5) = accConst(2);
    Z(6) = accConst(3);
    Z(7) = magConst(1);
    Z(8) = magConst(2);
    
    Y = Z - y;

	% S(k) = H(k)*P(k|k-1)*H(k)' + R(k)
	S = H*P*H' + R;
	% K(k) = P(k|k-1)*H(k)'*S(k)^-1
	K = P*H'/S; 
	% X(k|k) = X(k|k-1) + K(k)*Y(k)
	X = X + K*Y;
	% P(k|k) = P(k|k-1) - K(k)*H(k)*P(k|k-1);
	P = P - K*H*P;
    
    %% Add delta state
    X = X + DeltaX;
    
    q_norm = sqrt(X(ID_Q0)^2+X(ID_Q1)^2+X(ID_Q2)^2+X(ID_Q3)^2);
	X(ID_Q0:ID_Q3) = X(ID_Q0:ID_Q3)/q_norm;
    
    %% save result
    res_x(:,i) = X; 
    [roll(i), pitch(i), yaw(i)] = quad2euler(X(ID_Q0), X(ID_Q1), X(ID_Q2), X(ID_Q3), 'deg');
    
    bias(:,i) = X(ID_GX_BIAS:ID_AZ_BIAS);
end

%% plot result
index = 1:N;

% figure
% plot(index, bias(1,:));
% legend('gx bias')
% figure
% plot(index, bias(2,:));
% legend('gy bias')
% figure
% plot(index, bias(3,:));
legend('gz bias')

figure
grid on;
plot(index, LogField(:,roll_col), index, roll, 'LineWidth', 1.5);
legend('Roll', 'EKF.Roll')
figure
plot(index, LogField(:,pitch_col), index, pitch, 'LineWidth', 1.5);
legend('Pitch', 'EKF.Pitch')
figure
plot(index, LogField(:,yaw_col), index, yaw, 'LineWidth', 1.5);
legend('Yaw', 'EKF.Yaw')
figure
plot(index, LogField(:,gps_x_col), index, res_x(ID_X,:), 'LineWidth', 1.5);
legend('X', 'EKF.X')
figure
plot(index, LogField(:,gps_y_col), index, res_x(ID_Y,:), 'LineWidth', 1.5);
legend('Y', 'EKF.Y')
figure 
plot(index, LogField(:,baro_z_col), index, res_x(ID_Z,:), 'LineWidth', 1.5);
legend('Z', 'EKF.Z')
figure
plot(index, LogField(:,gps_vx_col), index, res_x(ID_VX,:), 'LineWidth', 1.5);
legend('VX', 'EKF.VX')
figure
plot(index, LogField(:,gps_vy_col), index, res_x(ID_VY,:), 'LineWidth', 1.5);
legend('VY', 'EKF.VY')
figure
plot(index, LogField(:,baro_vz_col), index, res_x(ID_VZ,:), 'LineWidth', 1.5);
legend('VZ', 'EKF.VZ')
