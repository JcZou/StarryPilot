% Ellipsoid fitting, using least-square method
% Author: Jiachi Zou
% @StarryPilot 

%% config sensor type
sensor = 'acc';
iteration_fit = 1;

%% read sensor data
filename = uigetfile('*.*');
fid = fopen(filename);

Data = fscanf(fid, '%f %f %f', [3 inf]);
Data = Data';
fclose(fid);
x = Data(:,1);
y = Data(:,2);
z = Data(:,3);

%% do ellipsoid fit
if strcmp(sensor,'acc')
    [ rotM, bias ] = ellipsoid_fit_run( x, y, z, 0, iteration_fit );
elseif strcmp(sensor,'mag')
    [ rotM, bias ] = ellipsoid_fit_run( x, y, z, 1, iteration_fit );
else
    error('unknown sensor type');
end