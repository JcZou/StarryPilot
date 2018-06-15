function [ y ] = fhan( x1, x2, r, h )
%UNTITLED2 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��

d = r * h^2;
a0 = h * x2;
y = x1 + a0;
a1 = sqrt( d * (d + 8*abs(y)) );
a2 = a0 + sign(y)*(a1 - d)/2;
sy = (sign(y + d) - sign(y - d))/2;
a = (a0 + y - a2)*sy + a2;
sa = (sign(a + d) - sign(a - d))/2;
y = -r*(a/d - sign(a))*sa - r*sign(a);

end

