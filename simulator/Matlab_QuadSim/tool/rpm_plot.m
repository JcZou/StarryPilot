throttle = 0.2:0.1:0.7;
omega = [234.7816909783, 303.1636910714, 372.3834492055, 447.1533543609, 519.0958261282, 592.9232534875];

x = 0.2:0.01:0.7;
p1 = polyfit(throttle, omega, 1);
y1 = polyval(p1,x);

figure;
plot(throttle, omega, 'blacko', x, y1, 'b');
xlabel('throttle');
ylabel('\omega (rad/s)');
poly1 = sprintf('y=%.3fx+%.3f', p1(1), p1(2));
legend('measure point',poly1, 'Location','northwest');
grid on;