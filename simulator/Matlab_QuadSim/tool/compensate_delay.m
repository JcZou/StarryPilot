clear all
clc
h = 0.004;
period = 1;
motor_sys = ss(tf(1, [0.025, 1]));
motor_sysd = c2d(motor_sys, h);
motor_x_state = 0;
state = 0;

roll_td.v1 = 0;
roll_td.v2 = 0;
roll_td.r0 = 20000;
roll_td.h = h*0.95;

u = step_gen(0, h, period);
x = 0:h:period;
for k = 1:length(x)
%     [ state, mw2_delay ] = rotor_dynamic( motor_sysd, state, u(k) );
    roll_td = adrc_td(roll_td, u(k));
    vv(k,1) = u(k);
    vv(k,2) = roll_td.v1;
    vv(k,3) = roll_td.v2;
    if u(k) > 0
        aa = u(k);
    end
    out = roll_td.v1 + 0.025*roll_td.v2;
    vv(k,4) = out;
    [ motor_x_state, mw2_delay ] = rotor_dynamic( motor_sysd, motor_x_state, out );
    x2(k) = mw2_delay;
end
figure
plot(x,u,x,x2);
legend('u','x2');
figure
plot(x,vv(:,1),x,vv(:,2),x,vv(:,4));
legend('v','v1','U');
figure
plot(x,vv(:,3));
legend('v2')