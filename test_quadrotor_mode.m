%% This script will fly the quadrotor in a fixed roll pitch and yaw angles maintaining a fixed height
%Author : Eranga Fernando
%Email  : hctef2@mun.ca, hcteranga@gmail.com
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%     _   _   _   _   _   _     %%%
%%%    / \ / \ / \ / \ / \ / \    %%%
%%%   ( H | C | T | E | F | 2 )   %%%
%%%    \_/ \_/ \_/ \_/ \_/ \_/    %%%
%%%                               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;
clc;

%% Simulation parameters
% traj_no = 1.0; % One direction
% traj_no = 2.0; % Stationary
traj_no = 2.0; % Stationary
sys_parameters;
dt = 1/200;
% Includeing noise
noise = 0;

if traj_no == 1
    T = 30; % 30 Seconds
    length = T/dt;
    theta = zeros(1,length);
    phi = zeros(1,length);
    psi = zeros(1,length);    
    theta(2001:3200) = +0.1 -0.1*cos([1:1200]*pi/600);
    h_d = 3;
end

if traj_no == 2
    T = 100; %100 Seconds
    length = T/dt;
    theta = zeros(1,length);
    phi = zeros(1,length);
    psi = zeros(1,length);
    h_d = 3;
end

%% Defining the quadrotor
% Initial state
X_int = [2, 2, 3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0];
Q1 = Quadrotor(1,X_int,param,dt,3,noise,length);
m = Q1.quad.m;
g = Q1.quad.g;

% Gains for PID controller
Kp = 3.5;
Kd = 2.4;



%% Controlling the trajectory
for i = 1:length
    i;
    if (i == 1)        
        Q1.kinematic_sim(0,theta(i),0,m*g);
    else
        %% Thrust calculation
        u = m*g + Kp*(h_d - Q1.groundtruth.p(3,i-1)) - Kd*Q1.groundtruth.p_dot(3,i-1);
        if (u > 7)
            u = 7;
        elseif (u < 4)
            u = 4;
        end
        Q1.kinematic_sim(phi(i),theta(i),psi(i),u);
    end

end
% Visualize
Q1.visualize

