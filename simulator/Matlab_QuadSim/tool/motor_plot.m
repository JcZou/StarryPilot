throttle = 0.1:0.1:0.9;
dmass = [24 64 112 172 254 343 462 572 710];    % change of mass (g)
thrust = dmass/1000*9.8066;    % change into newton

throttle_rpm = 0.2:0.1:0.7;
omega = [234.7816909783, 303.1636910714, 372.3834492055, 447.1533543609, 519.0958261282, 592.9232534875];

for k = 1:length(thrust)-1
    gap(k) = thrust(k+1)-thrust(k);
end

x = 0:0.01:1;
p1 = polyfit(throttle_rpm, omega, 1);
y1 = polyval(p1,x);

p2 = polyfit(throttle, thrust, 2);
y2 = polyval(p2,x);

figure;
subplot(2,1,1,'FontSize',14);
plot(throttle_rpm, omega, 'blacko', x, y1, 'r');
ylabel('motor speed (rad/s)');
poly1 = sprintf('y=%.3fx+%.3f', p1(1), p1(2));
legend('measurement point',poly1, 'Location','northwest');
grid on;
subplot(2,1,2,'FontSize',14);
plot(throttle, thrust, 'blacko', x, y2, 'b');
xlabel('\Gamma');
ylabel('thrust (N)');
poly2 = sprintf('y=%.3fx^2+%.3fx+%.3f', p2(1), p2(2), p2(3));
legend('measurement point', poly2, 'Location','northwest');
% axis([0 1 0 7])
grid on;

% load motor_delay.txt;
% figure;
% plot(motor_delay(:,1), motor_delay(:,2), '*-');
% xlabel('time(s)');
% ylabel('thrust(g)');