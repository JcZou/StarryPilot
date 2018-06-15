function h = filledCircle(center,r,N,color)
%---------------------------------------------------------------------------------------------
% FILLEDCIRCLE Filled circle drawing
% 
% filledCircle(CENTER,R,N,COLOR) draws a circle filled with COLOR that 
% has CENTER as its center and R as its radius, by using N points on the 
% periphery.
%
% Usage Examples,
%
% filledCircle([1,3],3,1000,'b'); 
% filledCircle([2,4],2,1000,'r');
%
% Sadik Hava <sadik.hava@gmail.com>
% May, 2010
%
% Inspired by: circle.m [Author: Zhenhai Wang]
%---------------------------------------------------------------------------------------------

THETA=linspace(0,2*pi,N);
RHO=ones(1,N)*r;
[X,Y] = pol2cart(THETA,RHO);
X=X+center(1);
Y=Y+center(2);
h=fill(X,Y,color);
%axis square;
end