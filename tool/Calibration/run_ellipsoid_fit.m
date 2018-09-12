% Ellipsoid fitting algorithm
% Author: Jiachi Zou
% 2017/10/03

clear all;
%% create the test data:
% radii
a = 2;
b = 5;
c = 2;
[ s, t ] = meshgrid( -pi : 2*pi/10 : pi, -pi : 2*pi/10 : pi );
x = a * cos(s) .* cos( t );
y = b * cos(s) .* sin( t );
z = c * sin(s);
% rotation
ang = pi/180*45;
xt = x * cos( ang ) + y * sin( ang );
yt = -x * sin( ang ) + y * cos( ang );
% translation
shiftx = 0;
shifty = 0;
shiftz = 0;
x = xt + shiftx;
y = yt + shifty;
z = z  + shiftz;

% add noise:
noiseIntensity = 0.1; %
dx = randn( size( s ) ) * noiseIntensity;
dy = randn( size( s ) ) * noiseIntensity;
dz = randn( size( s ) ) * noiseIntensity;
x = x + dx;
y = y + dy;
z = z + dz;
x = x(:);
y = y(:);
z = z(:);

%% read data from file
fid = fopen('acc.dat');    % change the file name to your test file
refr = 9.8; % reference radius

Data = fscanf(fid, '%f %f %f', [3 inf]);
Data = Data';
fclose(fid);
x = Data(:,1);
y = Data(:,2);
z = Data(:,3);

%% do the fitting
[ center, radii, evecs, v] = my_ellipsoid_fit( [ x y z ] );
g_mat = zeros(3, 3);
g_mat(1,1) = 1/radii(1)*refr;
g_mat(2,2) = 1/radii(2)*refr;
g_mat(3,3) = 1/radii(3)*refr;
% g_mat will rotate axis, so we need rotate axis back
trans_mat = evecs*g_mat*pinv(evecs);

fprintf( 'Ellipsoid center: %.5g %.5g %.5g\n', center );
fprintf( 'Transform matrix:\n' );
fprintf( '%.5g %.5g %.5g\n%.5g %.5g %.5g\n%.5g %.5g %.5g\n', ...
    trans_mat(1), trans_mat(4), trans_mat(7), trans_mat(2), trans_mat(5), trans_mat(8), trans_mat(3), trans_mat(6), trans_mat(9) );
fprintf( 'Ellipsoid radii: %.5g %.5g %.5g\n', radii );
fprintf( 'Ellipsoid evecs:\n' );
fprintf( '%.5g %.5g %.5g\n%.5g %.5g %.5g\n%.5g %.5g %.5g\n', ...
    evecs(1), evecs(4), evecs(7), evecs(2), evecs(5), evecs(8), evecs(3), evecs(6), evecs(9) );
fprintf( 'Algebraic form:\n' );
fprintf( '%.5g ', v );
%fprintf( '\nAverage deviation of the fit: %.5f\n', sqrt( chi2 / size( x, 1 ) ) );
fprintf( '\n' );

%% draw data points
figure,
plot3( x, y, z, '.r' );
hold on;

%% draw fit ellipsoid
mind = min( [ x y z ] );
maxd = max( [ x y z ] );
nsteps = 50;
step = ( maxd - mind ) / nsteps;
[ X, Y, Z ] = meshgrid( linspace( mind(1) - step(1), maxd(1) + step(1), nsteps ), linspace( mind(2) - step(2), maxd(2) + step(2), nsteps ), linspace( mind(3) - step(3), maxd(3) + step(3), nsteps ) );

Ellipsoid = v(1) *X.*X +   v(2) * Y.*Y + v(3) * Z.*Z + ...
          2*v(4) *X.*Y + 2*v(5)*X.*Z + 2*v(6) * Y.*Z + ...
          2*v(7) *X    + 2*v(8)*Y    + 2*v(9) * Z;
p = patch( isosurface( X, Y, Z, Ellipsoid, -v(10) ) );
hold off;
set( p, 'FaceColor', 'g', 'EdgeColor', 'none' );
view( -70, 40 );
%axis vis3d equal;
axis equal;
camlight;
lighting phong;
xlabel('X'); ylabel('Y');zlabel('Z'); grid on;

xlim = get(gca,'Xlim');
ylim = get(gca,'Ylim');
zlim = get(gca,'Xlim');

%% calibrate data
XC=x-center(1); YC=y-center(2); ZC=z-center(3); % translate to (0,0,0)
%XYZC=[XC,YC,ZC]*evecs; % rotate to XYZ axes
XYZC=[XC,YC,ZC]*trans_mat;
XC=XYZC(:,1);
YC=XYZC(:,2);
ZC=XYZC(:,3);
%% draw calibrated data points
figure;
plot3(XC,YC,ZC,'r*');

%% draw X axis
tx = linspace(0,xlim(2),50)';
ty = zeros(1,50)';
tz = zeros(1,50)';
XC=tx-center(1); YC=ty-center(2); ZC=tz-center(3); % translate to (0,0,0)
XYZC=[XC,YC,ZC]*trans_mat;
XC=XYZC(:,1);
YC=XYZC(:,2);
ZC=XYZC(:,3);
hold on;
plot3(XC,YC,ZC,'y*');

%% draw Y axis
tx = zeros(1,50)';
ty = linspace(0,ylim(2),50)';
tz = zeros(1,50)';
XC=tx-center(1); YC=ty-center(2); ZC=tz-center(3); % translate to (0,0,0)
XYZC=[XC,YC,ZC]*trans_mat;
XC=XYZC(:,1);
YC=XYZC(:,2);
ZC=XYZC(:,3);
hold on;
plot3(XC,YC,ZC,'g*');
xlabel('X'); ylabel('Y');zlabel('Z'); axis equal; grid on;

%% draw Z axis
tx = zeros(1,50)';
ty = zeros(1,50)';
tz = linspace(0,zlim(2),50)';
XC=tx-center(1); YC=ty-center(2); ZC=tz-center(3); % translate to (0,0,0)
XYZC=[XC,YC,ZC]*trans_mat;
XC=XYZC(:,1);
YC=XYZC(:,2);
ZC=XYZC(:,3);
hold on;
plot3(XC,YC,ZC,'b*');