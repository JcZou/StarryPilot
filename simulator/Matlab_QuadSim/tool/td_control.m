clear all
h = 0.01;
period = 3;
time = 0 : h : period;
r = step_gen(1, h, period)*10;
e = r;
v1 = 0;
v2 = 0;
err = 10;
for i = 1:length(r)
    v1 = v1 + h*v2;
    v2 = v2 + h*fhan(-err,v2,50,5*h);
    ss(i,1) = v1;
    ss(i,2) = v2;
    err = err - err*0.1;
    if err < 0
        err = 0;
    end
end
figure
plot(time, r, time, ss(:,1));
figure
plot(time, ss(:,2));