function unitstep = step_gen(start, sample_time, period)
%UNTITLED2 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��

t = (0 : sample_time : period)';
unitstep = (t>=start)*1;

end

