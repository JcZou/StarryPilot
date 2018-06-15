function [ th ] = throttle_mix( base_th, u, airframe )
%UNTITLED9 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��

    if airframe == 'X'
        th = throttle_mix_X( base_th, u );
    else
        disp('unknow airframe type');
    end

    % constrain throttle between 0 and 1
    th = constrain(th, 0, 1);
end

function [ out ] = throttle_mix_X( base_th, u )
%UNTITLED9 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��

    motor_mat = [-1 1 1;1 -1 1;1 1 -1;-1 -1 -1];
    out = base_th + motor_mat * u;
end

