%% Parameters of the quadrotor and the simulation
%% Physical Parameters
param.k = 6.41e-6;      % Thrust coefficient
param.k_pp = 0.00011;   % Perpendicular drag coefficient, should be multiplied by propeller speed
param.k_pa = 0.00023;   % Parallel drag coefficient
param.b = 6.41e-6;      % Thrust coefficient
param.d = 9.31e-7;      % Drag coefficient
param.l = 0.34;         % Distance between opposite propellers
param.I = diag([0.00367556974 0.00365031574 0.00703095144]);    % Moment of inertia
param.m = 0.56;         % Mass of the quadrotor
param.g = 9.81;         % Gravitational acceleration


%% Simulation parameters
param.dt = 1/200;


% Measurement parameters
param.sim_sigma_Ax = 2.0000e-3; % [ m / s^2 / sqrt(Hz) ]
param.sim_sigma_Ay = 2.0000e-3; % [ m / s^2 / sqrt(Hz) ]
param.sim_sigma_Az = 2.0000e-3; % [ m / s^2 / sqrt(Hz) ]
param.sim_sigma_Gx = 1.6968e-4; % [ rad / s / sqrt(Hz) ]
param.sim_sigma_Gy = 1.6968e-4; % [ rad / s / sqrt(Hz) ]
param.sim_sigma_Gz = 1.6968e-4; % [ rad / s / sqrt(Hz) ]
param.sim_sigma_bg = 1.9393e-5; % [ rad / s^2 / sqrt(Hz) ]
param.sim_sigma_ba = 3.00e-3; % [ m / s^3 / sqrt(Hz) ]
param.sim_sigma_h = 0.005; % m
param.sim_sigma_prop = 20;
param.sim_sigma_rpy = [0.0043 0.0043 0.0043];
param.sim_sigma_uwb = 0.01;


param.sigma_roll = 0.0043;% 0.5 deg = 0.5*(pi/180)
param.sigma_pitch = 0.0043;
param.sigma_yaw = 0.0043;
param.no_beacons = 3;

% Beacon locations : [x y z]'
param.beacon_loc = [1 10 2; 5 5 1; 0 0 3]';




