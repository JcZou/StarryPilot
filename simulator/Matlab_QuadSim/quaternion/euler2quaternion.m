function [ q ] = euler2quaternion( e )
%UNTITLED4 �˴���ʾ�йش˺����ժҪ
%   �˴���ʾ��ϸ˵��

    cr = cos(e(1) * 0.5);
    sr = sin(e(1) * 0.5);	
    cp = cos(e(2) * 0.5);
    sp = sin(e(2) * 0.5);
    cy = cos(e(3) * 0.5);
    sy = sin(e(3) * 0.5);
    
	q(1) = cy * cr * cp + sy * sr * sp;
	q(2) = cy * sr * cp - sy * cr * sp;
	q(3) = cy * cr * sp + sy * sr * cp;
	q(4) = sy * cr * cp - cy * sr * sp;

    q = reshape(q, [4, 1]);
end

