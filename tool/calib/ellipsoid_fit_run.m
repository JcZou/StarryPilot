function [ rotM, bias ] = ellipsoid_fit_run( x, y, z, rotated_fit, iteration_fit )

%% ellipsoid fitting
if iteration_fit
    [ center, radii, evecs, v] = ellipsoid_iteration_fit( [ x y z ], rotated_fit);
else
    [ center, radii, evecs, v] = ellipsoid_fit( [ x y z ], rotated_fit);
end

g_mat = diag(1./radii);
% g_mat will rotate axis, so we need rotate axis back
trans_mat = evecs*g_mat*pinv(evecs);
% main the norm of input data
trans_mat = trans_mat'/norm(trans_mat);

bias = center;
rotM = trans_mat;

%% visualize fitting result
figure,
subplot(1,2,1);
% display original data
plot3( x, y, z, '.r' );
hold on;

% display fit ellipsoid
mind = min( [ x y z ] );
maxd = max( [ x y z ] );
nsteps = 50;
step = ( maxd - mind ) / nsteps;
[ X, Y, Z ] = meshgrid( linspace( mind(1) - step(1), maxd(1) + step(1), nsteps ), linspace( mind(2) - step(2), maxd(2) + step(2), nsteps ), linspace( mind(3) - step(3), maxd(3) + step(3), nsteps ) );

Ellipsoid = v(1) *X.*X +   v(2) * Y.*Y + v(3) * Z.*Z + ...
          2*v(4) *X.*Y + 2*v(5)*X.*Z + 2*v(6) * Y.*Z + ...
          2*v(7) *X    + 2*v(8)*Y    + 2*v(9) * Z;
      
fit_val = v(1) *x.*x +   v(2) * y.*y + v(3) * z.*z + ...
          2*v(4) *x.*y + 2*v(5)*x.*z + 2*v(6) * y.*z + ...
          2*v(7) *x    + 2*v(8)*y    + 2*v(9) * z;
isovalue = mean(fit_val);
p = patch( isosurface( X, Y, Z, Ellipsoid, isovalue ) );
hold off;
set( p, 'FaceColor', 'g', 'EdgeColor', 'none' );
view( -70, 40 );
axis equal;
camlight;
lighting phong;
xlabel('X'); ylabel('Y');zlabel('Z'); grid on;

xlim = get(gca,'Xlim');
ylim = get(gca,'Ylim');
zlim = get(gca,'Xlim');

% display calibrated data
XC=x-center(1); YC=y-center(2); ZC=z-center(3); % translate to (0,0,0)
XYZC=trans_mat*[XC,YC,ZC]';
XC=XYZC(1,:);
YC=XYZC(2,:);
ZC=XYZC(3,:);
fitting_radius = sqrt(XC.*XC + YC.*YC + ZC.*ZC);
refr = mean(fitting_radius);

% display calibrated data points
subplot(1,2,2);
plot3(XC,YC,ZC,'r.');

% %% draw X axis
% tx = linspace(0,xlim(2),50)';
% ty = zeros(1,50)';
% tz = zeros(1,50)';
% XC=tx-center(1); YC=ty-center(2); ZC=tz-center(3); % translate to (0,0,0)
% XYZC=trans_mat*[XC,YC,ZC]';
% XC=XYZC(1,:);
% YC=XYZC(2,:);
% ZC=XYZC(3,:);
% hold on;
% plot3(XC,YC,ZC,'y-', 'LineWidth', 2);
% 
% %% draw Y axis
% tx = zeros(1,50)';
% ty = linspace(0,ylim(2),50)';
% tz = zeros(1,50)';
% XC=tx-center(1); YC=ty-center(2); ZC=tz-center(3); % translate to (0,0,0)
% XYZC=trans_mat*[XC,YC,ZC]';
% XC=XYZC(1,:);
% YC=XYZC(2,:);
% ZC=XYZC(3,:);
% hold on;
% plot3(XC,YC,ZC,'g-', 'LineWidth', 2);
% 
% %% draw Z axis
% tx = zeros(1,50)';
% ty = zeros(1,50)';
% tz = linspace(0,zlim(2),50)';
% XC=tx-center(1); YC=ty-center(2); ZC=tz-center(3); % translate to (0,0,0)
% XYZC=trans_mat*[XC,YC,ZC]';
% XC=XYZC(1,:);
% YC=XYZC(2,:);
% ZC=XYZC(3,:);
% hold on;
% plot3(XC,YC,ZC,'b-', 'LineWidth', 2);

% display reference sphere
[x y z] = sphere();
surface(refr*x, refr*y, refr*z, 'FaceColor', 'none');
xlabel('X'); ylabel('Y');zlabel('Z'); axis equal; grid on;

%% print fiting variables
fprintf( 'Ellipsoid bias: %.5g %.5g %.5g\n', center );
fprintf( 'Rotation matrix:\n' );
fprintf( '%.5g %8.5g %8.5g\n%.5g %8.5g %8.5g\n%.5g %8.5g %8.5g\n', ...
    trans_mat(1), trans_mat(4), trans_mat(7), trans_mat(2), trans_mat(5), trans_mat(8), trans_mat(3), trans_mat(6), trans_mat(9) );
fprintf( 'Ellipsoid radius: %.5g %.5g %.5g\n', radii );
fprintf( 'Sphere radius: %.5g\n', refr );
fprintf( 'Algebraic form:\n' );
fprintf( '%.5g ', v );
fprintf( '\n' );

end

