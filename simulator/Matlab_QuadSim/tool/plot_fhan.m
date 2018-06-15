
% h1 decide the wide of linear part (x~[-h1,h1])
% r decide the maximal and minimal value of fhan
clear all;
c = 0.2;
h = 0.004;
r = 0.4;
[X,Y] = meshgrid(-1:0.05:1);
% Z = fhan( X, Y, 1, 0.1 );
for i = 1:length(X)
    for j = 1:length(Y)
        Z(i,j) = fhan( X(i,j), c*Y(i,j), r, 200*0.004);
    end
end 
% figure('FontSize',12);
surf(X,Y,Z);
xlabel('x1');
ylabel('x2');