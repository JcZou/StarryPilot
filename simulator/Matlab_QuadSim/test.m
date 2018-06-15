%     % for testing
%     if Var.sim_time >= 3.2
%         Var.sp_att(1) = 0/50*25/180*pi;
%     elseif Var.sim_time >= 3
%         Var.sp_att(1) = 50/50*25/180*pi;
%     elseif Var.sim_time >= 2
%         Var.sp_att(1) = -50/50*25/180*pi;
%     elseif Var.sim_time >= 1
%         Var.sp_att(1) = 50/50*25/180*pi;
%     end

%     if Var.sim_time >= 3
%         Var.sp_alt = -0.5;
%     else
%         Var.sp_alt = -1;
%     end

%     if Var.sim_time >=1 
%         Var.sp_att(1) = 50/50*25/180*pi*sin(Var.sim_time/Var.h*pi/200);
%     end

%     if Var.sim_time >= 6
%         set(Var.disturb_hd, 'value', 0);
%     elseif Var.sim_time >= 3
%         set(Var.disturb_hd, 'value', 1);
%     end