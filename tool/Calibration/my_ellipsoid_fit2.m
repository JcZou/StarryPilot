function [ofs,gain,rotM,u]=my_ellipsoid_fit2(XYZ)
% Fit an (non)rotated ellipsoid or sphere to a set of xyz data points
% XYZ: N(rows) x 3(cols), matrix of N data points (x,y,z)
% optional flag f, default to 0 (fitting of rotated ellipsoid)
x=XYZ(:,1); y=XYZ(:,2); z=XYZ(:,3); 

len = length(x);
v = zeros(9,1);
% P = eye(9);
P = diag([10,10,10,1,1,1,1,1,1]);
R = 0.001;
for i = 1:len
    H = [x(i).*x(i), y(i).*y(i), z(i).*z(i), 2*x(i).*y(i),2*x(i).*z(i),2*y(i).*z(i), 2*x(i),2*y(i),2*z(i)];
    
    Y = 1 - H*v;
    % S(k) = H(k)*P(k|k-1)*H(k)' + R(k)
    S = H*P*H' + R;
    % K(k) = P(k|k-1)*H(k)'*S(k)^-1
    K = P*H'/S; 
    % X(k|k) = X(k|k-1) + K(k)*Y(k)
    v = v + K*Y;
    % P(k|k) = P(k|k-1) - K(k)*H(k)*P(k|k-1);
    P = P - K*H*P;
%     P = (eye(9)-K*H)*P*(eye(9)-K*H)'+K*R*K';
end

u = v;
A = [ v(1) v(4) v(5) v(7); v(4) v(2) v(6) v(8); v(5) v(6) v(3) v(9); v(7) v(8) v(9) -1 ];
ofs=-A(1:3,1:3)\[v(7);v(8);v(9)]; % offset is center of ellipsoid
Tmtx=eye(4); Tmtx(4,1:3)=ofs'; AT=Tmtx*A*Tmtx'; % ellipsoid translated to (0,0,0)
[rotM ev]=eig(AT(1:3,1:3)/-AT(4,4)); % eigenvectors (rotation) and eigenvalues (gain)
gain=sqrt(1./diag(ev)); % gain is radius of the ellipsoid