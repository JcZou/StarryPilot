function  draw_quadrotor_3D( quad3D, state )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    global Var;
    
    set(quad3D.Arm1.Obj,'xdata',quad3D.Arm1.PosX,'ydata',quad3D.Arm1.PosY,'zdata',quad3D.Arm1.PosZ);
    set(quad3D.Arm2.Obj,'xdata',quad3D.Arm2.PosX,'ydata',quad3D.Arm2.PosY,'zdata',quad3D.Arm2.PosZ);
    set(quad3D.Rotor1.Obj,'xdata',quad3D.Rotor1.PosX,'ydata',quad3D.Rotor1.PosY,'zdata',quad3D.Rotor1.PosZ);
    set(quad3D.Rotor2.Obj,'xdata',quad3D.Rotor2.PosX,'ydata',quad3D.Rotor2.PosY,'zdata',quad3D.Rotor2.PosZ);
    set(quad3D.Rotor3.Obj,'xdata',quad3D.Rotor3.PosX,'ydata',quad3D.Rotor3.PosY,'zdata',quad3D.Rotor3.PosZ);
    set(quad3D.Rotor4.Obj,'xdata',quad3D.Rotor4.PosX,'ydata',quad3D.Rotor4.PosY,'zdata',quad3D.Rotor4.PosZ);
    set(Var.xline_hd, 'XData', [state.posI(1)-0.3 state.posI(1)+0.3], 'YData', [state.posI(2) state.posI(2)]);
    set(Var.yline_hd, 'XData', [state.posI(1) state.posI(1)], 'YData', [state.posI(2)-0.3 state.posI(2)+0.3]);
end

