function [ new_quad3D ] = update_quadrotor_3D( quad3D, state )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

    % update attitude
    R = rotation_mat_body2inertia(state.att.q);
    [row col] = size(quad3D.Arm1.X);
    for c = 1:col
        P1 = [quad3D.Arm1.X(:,c)'; quad3D.Arm1.Y(:,c)'; quad3D.Arm1.Z(:,c)'];
        P2 = R * P1;
        quad3D.Arm1.PosX(:,c) = P2(1,:)';
        quad3D.Arm1.PosY(:,c) = P2(2,:)';
        quad3D.Arm1.PosZ(:,c) = P2(3,:)';
        
        P1 = [quad3D.Arm2.X(:,c)'; quad3D.Arm2.Y(:,c)'; quad3D.Arm2.Z(:,c)'];
        P2 = R * P1;
        quad3D.Arm2.PosX(:,c) = P2(1,:)';
        quad3D.Arm2.PosY(:,c) = P2(2,:)';
        quad3D.Arm2.PosZ(:,c) = P2(3,:)';
    end
    
    [row col] = size(quad3D.Rotor1.X);
    for c = 1:col
        P1 = [quad3D.Rotor1.X(:,c)'; quad3D.Rotor1.Y(:,c)'; quad3D.Rotor1.Z(:,c)'];
        P2 = R * P1;
        quad3D.Rotor1.PosX(:,c) = P2(1,:)';
        quad3D.Rotor1.PosY(:,c) = P2(2,:)';
        quad3D.Rotor1.PosZ(:,c) = P2(3,:)';
        
        P1 = [quad3D.Rotor2.X(:,c)'; quad3D.Rotor2.Y(:,c)'; quad3D.Rotor2.Z(:,c)'];
        P2 = R * P1;
        quad3D.Rotor2.PosX(:,c) = P2(1,:)';
        quad3D.Rotor2.PosY(:,c) = P2(2,:)';
        quad3D.Rotor2.PosZ(:,c) = P2(3,:)';
        
        P1 = [quad3D.Rotor3.X(:,c)'; quad3D.Rotor3.Y(:,c)'; quad3D.Rotor3.Z(:,c)'];
        P2 = R * P1;
        quad3D.Rotor3.PosX(:,c) = P2(1,:)';
        quad3D.Rotor3.PosY(:,c) = P2(2,:)';
        quad3D.Rotor3.PosZ(:,c) = P2(3,:)';
        
        P1 = [quad3D.Rotor4.X(:,c)'; quad3D.Rotor4.Y(:,c)'; quad3D.Rotor4.Z(:,c)'];
        P2 = R * P1;
        quad3D.Rotor4.PosX(:,c) = P2(1,:)';
        quad3D.Rotor4.PosY(:,c) = P2(2,:)';
        quad3D.Rotor4.PosZ(:,c) = P2(3,:)';
    end
    
    % update position
    quad3D.Arm1.PosX = quad3D.Arm1.PosX + state.posI(1);
    quad3D.Arm1.PosY = quad3D.Arm1.PosY + state.posI(2);
    quad3D.Arm1.PosZ = quad3D.Arm1.PosZ + state.posI(3);
    quad3D.Arm2.PosX = quad3D.Arm2.PosX + state.posI(1);
    quad3D.Arm2.PosY = quad3D.Arm2.PosY + state.posI(2);
    quad3D.Arm2.PosZ = quad3D.Arm2.PosZ + state.posI(3);
    quad3D.Rotor1.PosX = quad3D.Rotor1.PosX + state.posI(1);
    quad3D.Rotor1.PosY = quad3D.Rotor1.PosY + state.posI(2);
    quad3D.Rotor1.PosZ = quad3D.Rotor1.PosZ + state.posI(3);
    quad3D.Rotor2.PosX = quad3D.Rotor2.PosX + state.posI(1);
    quad3D.Rotor2.PosY = quad3D.Rotor2.PosY + state.posI(2);
    quad3D.Rotor2.PosZ = quad3D.Rotor2.PosZ + state.posI(3);
    quad3D.Rotor3.PosX = quad3D.Rotor3.PosX + state.posI(1);
    quad3D.Rotor3.PosY = quad3D.Rotor3.PosY + state.posI(2);
    quad3D.Rotor3.PosZ = quad3D.Rotor3.PosZ + state.posI(3);
    quad3D.Rotor4.PosX = quad3D.Rotor4.PosX + state.posI(1);
    quad3D.Rotor4.PosY = quad3D.Rotor4.PosY + state.posI(2);
    quad3D.Rotor4.PosZ = quad3D.Rotor4.PosZ + state.posI(3);
    
    new_quad3D = quad3D;

end

