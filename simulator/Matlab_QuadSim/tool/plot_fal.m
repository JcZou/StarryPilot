clear all;
x = -1:0.01:1;
for i = 1:length(x)
    y(i) = fal(x(i), 2, 0.1 );
end
plot(x,y);