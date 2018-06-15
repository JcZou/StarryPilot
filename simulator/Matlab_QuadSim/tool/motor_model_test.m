clear all;
h = 0.005;
period = 3;
% motor system, first order
motor_sys = ss(tf(1, [0.1, 1]));
motor_sysd = c2d(motor_sys, h);

motor_x_state = 0;


u = step_gen(1, h, period);
x = 0:h:period;
for k = 1:length(x)
    [ motor_x_state, mw2_delay ] = rotor_dynamic( motor_sysd, motor_x_state, u(k) );
    x2(k) = mw2_delay;
end
plot(x,u,x,x2);