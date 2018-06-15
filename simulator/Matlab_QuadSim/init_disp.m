global Var;

fig = figure('units','normalized','position',[.325 .1 .6 .8],'name','Quadrotor AUS','numbertitle','off','color','w');
fig.WindowButtonDownFcn = @fig_click_cb;
fig.WindowKeyPressFcn = @fig_key_press_cb;
fig.WindowScrollWheelFcn = @fig_mouse_scroll_cb;
axes('units','normalized','position',[.02 .1 .6 .8]);
% axes('units','normalized','position',[.02 .02 .9 .9]);
axis equal
xlabel('X')
ylabel('Y')
set(gca,'XDir','reverse')
set(gca,'ZDir','reverse')

Quadrotor3D.Arm1.Obj = patch('xdata',Quadrotor3D.Arm1.X,'ydata',Quadrotor3D.Arm1.Y,'zdata',Quadrotor3D.Arm1.Z,'facealpha',...
                        Quadrotor3D.Arm1.Alpha,'facecolor',Quadrotor3D.Arm1.C);
Quadrotor3D.Arm2.Obj = patch('xdata',Quadrotor3D.Arm2.X,'ydata',Quadrotor3D.Arm2.Y,'zdata',Quadrotor3D.Arm2.Z,'facealpha',...
                        Quadrotor3D.Arm2.Alpha,'facecolor',Quadrotor3D.Arm2.C);
Quadrotor3D.Rotor1.Obj = patch('xdata',Quadrotor3D.Rotor1.X,'ydata',Quadrotor3D.Rotor1.Y,'zdata',Quadrotor3D.Rotor1.Z,'facealpha',...
                        Quadrotor3D.Rotor1.Alpha,'facecolor',Quadrotor3D.Rotor1.C);
Quadrotor3D.Rotor2.Obj = patch('xdata',Quadrotor3D.Rotor2.X,'ydata',Quadrotor3D.Rotor2.Y,'zdata',Quadrotor3D.Rotor2.Z,'facealpha',...
                        Quadrotor3D.Rotor2.Alpha,'facecolor',Quadrotor3D.Rotor2.C);
Quadrotor3D.Rotor3.Obj = patch('xdata',Quadrotor3D.Rotor3.X,'ydata',Quadrotor3D.Rotor3.Y,'zdata',Quadrotor3D.Rotor3.Z,'facealpha',...
                        Quadrotor3D.Rotor3.Alpha,'facecolor',Quadrotor3D.Rotor3.C);
Quadrotor3D.Rotor4.Obj = patch('xdata',Quadrotor3D.Rotor4.X,'ydata',Quadrotor3D.Rotor4.Y,'zdata',Quadrotor3D.Rotor4.Z,'facealpha',...
                        Quadrotor3D.Rotor4.Alpha,'facecolor',Quadrotor3D.Rotor4.C);

Var.st_hd = uicontrol('units','normalized','position',[.9 .9 .08 .04],'style','text','fontsize',10,'HorizontalAlignment','left');
uicontrol('units','normalized','position',[.8 .9 .1 .04],'style','text','fontsize',10,'string','sim time:');
Var.x_hd = uicontrol('units','normalized','position',[.9 .85 .08 .04],'style','text','fontsize',10,'HorizontalAlignment','left');
uicontrol('units','normalized','position',[.8 .85 .1 .04],'style','text','fontsize',10,'string','X pos:');
Var.y_hd = uicontrol('units','normalized','position',[.9 .8 .08 .04],'style','text','fontsize',10,'HorizontalAlignment','left');
uicontrol('units','normalized','position',[.8 .8 .1 .04],'style','text','fontsize',10,'string','Y pos:');
Var.z_hd = uicontrol('units','normalized','position',[.9 .75 .08 .04],'style','text','fontsize',10,'HorizontalAlignment','left');
uicontrol('units','normalized','position',[.8 .75 .1 .04],'style','text','fontsize',10,'string','Z pos:');
Var.roll_hd = uicontrol('units','normalized','position',[.9 .7 .08 .04],'style','text','fontsize',10,'HorizontalAlignment','left');
uicontrol('units','normalized','position',[.8 .7 .1 .04],'style','text','fontsize',10,'string','roll:');
Var.pitch_hd = uicontrol('units','normalized','position',[.9 .65 .08 .04],'style','text','fontsize',10,'HorizontalAlignment','left');
uicontrol('units','normalized','position',[.8 .65 .1 .04],'style','text','fontsize',10,'string','pitch:');
Var.yaw_hd = uicontrol('units','normalized','position',[.9 .6 .08 .04],'style','text','fontsize',10,'HorizontalAlignment','left');
uicontrol('units','normalized','position',[.8 .6 .1 .04],'style','text','fontsize',10,'string','yaw:');

Var.motor1_hd = uicontrol('units','normalized','position',[.71 .9 .08 .04],'style','text','fontsize',10,'HorizontalAlignment','left');
uicontrol('units','normalized','position',[.61 .9 .1 .04],'style','text','fontsize',10,'string','motor 1:');
Var.motor2_hd = uicontrol('units','normalized','position',[.71 .85 .08 .04],'style','text','fontsize',10,'HorizontalAlignment','left');
uicontrol('units','normalized','position',[.61 .85 .1 .04],'style','text','fontsize',10,'string','motor 2:');
Var.motor3_hd = uicontrol('units','normalized','position',[.71 .8 .08 .04],'style','text','fontsize',10,'HorizontalAlignment','left');
uicontrol('units','normalized','position',[.61 .8 .1 .04],'style','text','fontsize',10,'string','motor 3:');
Var.motor4_hd = uicontrol('units','normalized','position',[.71 .75 .08 .04],'style','text','fontsize',10,'HorizontalAlignment','left');
uicontrol('units','normalized','position',[.61 .75 .1 .04],'style','text','fontsize',10,'string','motor 4:');

Var.th_hd = uicontrol('units','normalized','position',[.9 .45 .08 .04],'style','text','fontsize',10,'HorizontalAlignment','left');
uicontrol('units','normalized','position',[.8 .45 .1 .04],'style','text','fontsize',10,'string','sp alt:');
uicontrol('units','normalized','Style', 'slider','Min',0,'Max',100,'Value',0,'Position', [.8 .4 .18 .04],'Callback', @th_cb);

Var.sp_roll_hd = uicontrol('units','normalized','position',[.9 .35 .08 .04],'style','text','fontsize',10,'HorizontalAlignment','left');
uicontrol('units','normalized','position',[.8 .35 .1 .04],'style','text','fontsize',10,'string','sp roll:');
Var.sp_roll_slider = uicontrol('units','normalized','Style', 'slider','Min',-50,'Max',50,'Value',0,'Position', [.8 .3 .18 .04],'Callback', {@att_cb, 1});

Var.sp_pitch_hd = uicontrol('units','normalized','position',[.9 .25 .08 .04],'style','text','fontsize',10,'HorizontalAlignment','left');
uicontrol('units','normalized','position',[.8 .25 .1 .04],'style','text','fontsize',10,'string','sp pitch:');
uicontrol('units','normalized','Style', 'slider','Min',-50,'Max',50,'Value',0,'Position', [.8 .2 .18 .04],'Callback', {@att_cb, 2});

Var.sp_yaw_hd = uicontrol('units','normalized','position',[.9 .15 .08 .04],'style','text','fontsize',10,'HorizontalAlignment','left');
uicontrol('units','normalized','position',[.8 .15 .1 .04],'style','text','fontsize',10,'string','sp yaw:');
uicontrol('units','normalized','Style', 'slider','Min',-50,'Max',50,'Value',0,'Position', [.8 .1 .18 .04],'Callback', {@att_cb, 3});

Var.start_hd = uicontrol('units','normalized','Style', 'togglebutton','string','start','Position', [.02 .02 .08 .04]);
Var.disturb_hd = uicontrol('units','normalized','Style', 'togglebutton','string','disturbance','Position', [.7 .02 .08 .04]);
Var.fix_pos_hd = uicontrol('units','normalized','Style', 'togglebutton','string','fix pos','Position', [.8 .02 .08 .04]);
Var.record_hd = uicontrol('units','normalized','Style', 'togglebutton','string','record','Position', [.9 .02 .08 .04]);

Var.adrc_enable = uicontrol('units','normalized','Style', 'checkbox','string','ADRC','Position', [.02 .9 .08 .04]);
Var.att_eso = uicontrol('units','normalized','Style', 'checkbox','string','ATT ESO','Position', [.15 .9 .08 .04], 'Callback', @att_eso_cb);
Var.att_td = uicontrol('units','normalized','Style', 'checkbox','string','ATT TD','Position', [.25 .9 .08 .04], 'Callback', @att_td_cb);
Var.alt_eso = uicontrol('units','normalized','Style', 'checkbox','string','ALT ESO','Position', [.35 .9 .08 .04], 'Callback', @alt_eso_cb);
Var.alt_td = uicontrol('units','normalized','Style', 'checkbox','string','ALT TD','Position', [.45 .9 .08 .04], 'Callback', @alt_td_cb);

Var.xline_hd = line('LineWidth', 1, 'XData', [-0.3 0.3], 'YData', [0 0], 'Color', 'r');
Var.yline_hd = line('LineWidth', 1, 'XData', [0 0], 'YData', [-0.3 0.3], 'Color', 'b');
%uicontrol(fig, 'Style','pushbutton', 'String', 'reset', 'Callback', @reset);

axis(Var.axis)
view(30,30)
grid on
hold on

Var.fig_att_td = figure('units','normalized','position',[.0 .0 .3 .45], 'Name','TD state', 'Visible', 'off');
av1 = subplot(2,1,1,'FontSize',12);
hv = animatedline(av1, 'MaximumNumPoints',2000, 'Color', 'r', 'LineWidth',1.5);
hv1 = animatedline(av1, 'MaximumNumPoints',2000, 'Color', 'b', 'LineWidth',1.5);
hrad = animatedline(av1, 'MaximumNumPoints',2000, 'Color', 'm', 'LineWidth',1.5);
legend('v', 'v1', 'real angle', 'Location','southwest');
xlabel('simulation time (s)')
ylabel('angle (rad)')
grid on;

av2 = subplot(2,1,2,'FontSize',12);
hv2 = animatedline(av2, 'MaximumNumPoints',2000, 'Color', 'r', 'LineWidth',1.5);
hvel = animatedline(av2, 'MaximumNumPoints',2000, 'Color', 'b', 'LineWidth',1.5);
legend('v2', 'real vel', 'Location','southwest');
xlabel('simulation time (s)')
ylabel('angle velocity (rad/s)')
grid on;

Var.fig_att_eso = figure('units','normalized','position',[.0 .45 .3 .45], 'Name','ESO Estimated States', 'Visible', 'off');
if Var.adrc_order == 3
    ax = subplot(3,1,1);
    %ax.YLim = [-1 1]
    hz1 = animatedline(ax, 'MaximumNumPoints',2000, 'Color', 'r', 'LineWidth',1.5);
    hx1 = animatedline(ax, 'MaximumNumPoints',2000, 'Color', 'b', 'LineWidth',1.5);
    % hs1 = animatedline(ax, 'MaximumNumPoints',2000, 'Color', 'c');
    legend('z1', 'x1', 'Location','northwest');
    xlabel('simulation time (s)')
    ylabel('angle (rad)')
    grid on;


    ax2 = subplot(3,1,2);
    %ax2.YLim = [-3 3]
    hz2 = animatedline(ax2, 'MaximumNumPoints',2000, 'Color', 'r', 'LineWidth',1.5);
    hx2 = animatedline(ax2, 'MaximumNumPoints',2000, 'Color', 'b', 'LineWidth',1.5);
    % hs2 = animatedline(ax2, 'MaximumNumPoints',2000, 'Color', 'c');
    legend('z2', 'x2', 'Location','northwest');
    xlabel('simulation time (s)')
    ylabel('angular velocity (rad/s)')
    grid on;

    ax3 = subplot(3,1,3);
    %ax2.YLim = [-3 3]
    hz3 = animatedline(ax3, 'MaximumNumPoints',2000, 'Color', 'r', 'LineWidth',1.5);
    hx3 = animatedline(ax3, 'MaximumNumPoints',2000, 'Color', 'b', 'LineWidth',1.5);
    % hs3 = animatedline(ax3, 'MaximumNumPoints',2000, 'Color', 'c');
    if Var.adrc_cleso == 1
        legend('z3+s3', 'x3', 'Location','northwest');
    else
        legend('z3', 'x3', 'Location','northwest');
    end
    xlabel('simulation time (s)')
    ylabel('disturbances (rad/s/s)')
elseif Var.adrc_order == 2
    ax3 = subplot(3,1,1,'FontSize',14);
    %ax2.YLim = [-3 3]
    hx3 = animatedline(ax3, 'MaximumNumPoints',2000, 'Color', 'b', 'LineWidth',2);
    % hs3 = animatedline(ax3, 'MaximumNumPoints',2000, 'Color', 'c');
    legend('roll', 'Location','northwest');
    %xlabel('simulation time (s)')
    ylabel('\phi (rad)')
    grid on;
    
    ax = subplot(3,1,2,'FontSize',14);
    %ax.YLim = [-1 1]
    hz1 = animatedline(ax, 'MaximumNumPoints',2000, 'Color', 'r', 'LineWidth',2);
    hx1 = animatedline(ax, 'MaximumNumPoints',2000, 'Color', 'b', 'LineWidth',2, 'LineStyle', '-.');
    % hs1 = animatedline(ax, 'MaximumNumPoints',2000, 'Color', 'c');
    legend('z1', 'x1', 'Location','northwest');
    %xlabel('simulation time (s)')
    ylabel('\omega (rad/s)')
    grid on;


    ax2 = subplot(3,1,3,'FontSize',14);
    %ax2.YLim = [-3 3]
    hz2 = animatedline(ax2, 'MaximumNumPoints',2000, 'Color', 'r', 'LineWidth',2);
    hx2 = animatedline(ax2, 'MaximumNumPoints',2000, 'Color', 'b', 'LineWidth',2, 'LineStyle', '-.');
    % hs2 = animatedline(ax2, 'MaximumNumPoints',2000, 'Color', 'c');
    if Var.adrc_cleso == 1
        legend('z2+s2', 'x2', 'Location','northwest');
    else
        legend('z2', 'x2', 'Location','northwest');
    end
    xlabel('simulation time (s)')
    ylabel('F(t) (rad/s^2)')
    grid on;
end
grid on;

Var.fig_alt_eso = figure('units','normalized','position',[.1 .45 .3 .45], 'Name','Alt ESO Estimated States', 'Visible', 'off');
ax3 = subplot(3,1,1,'FontSize',14);
%ax2.YLim = [-3 3]
gx3 = animatedline(ax3, 'MaximumNumPoints',2000, 'Color', 'b', 'LineWidth',2);
gref = animatedline(ax3, 'MaximumNumPoints',2000, 'Color', 'r', 'LineWidth',2,  'LineStyle', '-.');
% hs3 = animatedline(ax3, 'MaximumNumPoints',2000, 'Color', 'c');
lgd = legend('alt', 'reference', 'Location','northwest');
lgd.FontSize = 10;
% xlabel('simulation time (s)')
ylabel('height (m)')
grid on;

ax = subplot(3,1,2,'FontSize',14);
%ax.YLim = [-1 1]
gz1 = animatedline(ax, 'MaximumNumPoints',2000, 'Color', 'r', 'LineWidth',2);
gx1 = animatedline(ax, 'MaximumNumPoints',2000, 'Color', 'b', 'LineWidth',2, 'LineStyle', '-.');
% hs1 = animatedline(ax, 'MaximumNumPoints',2000, 'Color', 'c');
lgd = legend('z1', 'x1','Location','northwest');
lgd.FontSize = 10;
% xlabel('simulation time (s)')
ylabel('v_z (m/s)')
grid on;

ax2 = subplot(3,1,3,'FontSize',14);
%ax2.YLim = [-3 3]
gz2 = animatedline(ax2, 'MaximumNumPoints',2000, 'Color', 'r', 'LineWidth',2);
gx2 = animatedline(ax2, 'MaximumNumPoints',2000, 'Color', 'b', 'LineWidth',2, 'LineStyle', '-.');
% hs2 = animatedline(ax2, 'MaximumNumPoints',2000, 'Color', 'c');
lgd = legend('z2', 'x2', 'Location','northwest');
lgd.FontSize = 10;
xlabel('simulation time (s)')
ylabel('F(t) (m/s^2)')
grid on;

Var.fig_alt_td = figure('units','normalized','position',[.1 0 .3 .45], 'Name','Alt TD state', 'Visible', 'off');
av1 = subplot(2,1,1,'FontSize',14);
gv = animatedline(av1, 'MaximumNumPoints',2000, 'Color', 'r', 'LineWidth',2, 'LineStyle', '-.');
gv1 = animatedline(av1, 'MaximumNumPoints',2000, 'Color', 'b', 'LineWidth',2);
galt = animatedline(av1, 'MaximumNumPoints',2000, 'Color', 'm', 'LineWidth',2);
legend('reference', 'v1', 'alt', 'Location','southwest');
% legend('reference', 'alt', 'Location','southwest');
% xlabel('simulation time (s)')
ylabel('height (m)')
grid on;

av2 = subplot(2,1,2,'FontSize',14);
gv2 = animatedline(av2, 'MaximumNumPoints',2000, 'Color', 'r', 'LineWidth',2);
gvel = animatedline(av2, 'MaximumNumPoints',2000, 'Color', 'b', 'LineWidth',2);
legend('v2', 'v_z', 'Location','southwest');
% legend('v_z', 'Location','southwest');
xlabel('simulation time (s)')
ylabel('velocity (m/s)')
grid on;


%filledCircle([1,3],3,1000,'b');

function my_callback(src, event)
    disp('move it')
end

function fig_click_cb(src, event)
    disp('fig_click_cb')
end

function fig_key_press_cb(src, event)
    disp('fig_key_press_cb')
end

function fig_mouse_scroll_cb(src, event)
%     disp('fig_mouse_scroll_cb')
%     zoom on;
%     zoom(2);
%     zoom off;
    count = event.VerticalScrollCount
    amount = event.VerticalScrollAmount
    zoom(1-count*0.1);
end

function th_cb(src,event)
    global Var;
    th = src.Value;
    Var.sp_alt = -th/100*2;
end

function att_cb(src,event, id)
    global Var;
    th = src.Value;
    if id == 1    
        Var.sp_att(1) = th/50*25/180*pi;
    elseif id == 2
        Var.sp_att(2) = th/50*25/180*pi;
    else
        Var.sp_att(3) = th/50*pi;
    end
end

function att_eso_cb(src,event)
    global Var;
    if src.Value == 1
        Var.fig_att_eso.Visible = 'on';
    else
        Var.fig_att_eso.Visible = 'off';
    end
end

function att_td_cb(src,event)
    global Var;
    if src.Value == 1
        Var.fig_att_td.Visible = 'on';
    else
        Var.fig_att_td.Visible = 'off';
    end
end

function alt_eso_cb(src,event)
    global Var;
    if src.Value == 1
        Var.fig_alt_eso.Visible = 'on';
    else
        Var.fig_alt_eso.Visible = 'off';
    end
end

function alt_td_cb(src,event)
    global Var;
    if src.Value == 1
        Var.fig_alt_td.Visible = 'on';
    else
        Var.fig_alt_td.Visible = 'off';
    end
end
