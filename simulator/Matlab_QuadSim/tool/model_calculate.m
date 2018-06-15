
Hub.m = 0.430;
Hub.R = 0.05;
Hub.H = 0.1;

Motor.m = 0.084;
Motor.r2 = 0.015;
Motor.h = 0.025;
Motor.d = 0.255;

Arm.m = 0.015;
Arm.r1 = 0.0075;
Arm.L = 0.22;

ESC.m = 0.015;
ESC.a = 0.02;
ESC.b = 0.05;

Ixx_hub = 1/4*Hub.m*Hub.R^2 + 1/12*Hub.m*Hub.H^2 + Hub.m*(0.5*Hub.H)^2;
Iyy_hub = Ixx_hub;
Izz_hub = 1/2*Hub.m*Hub.R^2;

dp = Motor.d * sin(pi/4);
Ixx_motor = 4*(1/4*Motor.m*Motor.r2^2 + 1/3*Motor.m*Motor.h^2 + Motor.m*dp^2);
Iyy_motor = Ixx_motor;
Izz_motor = 4*(1/2*Motor.m*Motor.r2^2 + Motor.m*Motor.d^2);

arm_m = Arm.m + ESC.m;
arm_r = Hub.R + 0.5*Arm.L;
Ixx_arm = 4*(1/2*arm_m*Arm.r1^2 + 1/12*arm_m*sin(pi/4)^2*(Arm.L^2-3*Arm.r1^2) + arm_m*arm_r^2);
Iyy_arm = Ixx_arm;
Izz_arm = 4*(1/4*arm_m*Arm.r1^2 + 1/3*arm_m*Arm.L^2 + arm_m*arm_r^2);

Ixx = Ixx_hub + Ixx_motor + Ixx_arm;
Iyy = Iyy_hub + Iyy_motor + Iyy_arm;
Izz = Izz_hub + Izz_motor + Izz_arm;

Mass = Hub.m + 4*Motor.m + 4*arm_m;