%%QUADROTOR models the kinematics and the dynamics of the quadrotor
%   Quadrotor class can model the quadrotor as a kinematic model or a 
%   dynamic model. Coordinate system used in this model consider the 
%   upward direction as positive Z. Right hand coordinate system is used
%   (NWU). The class can mode the gyroscope biases and accelerometer 
%   biases as well. It is controlled by bias_on variable
%
%
%   Quadrotor frame is defined as follow
%       Motor 1 : Front (CCW)
%       Motor 2 : Left (CW)
%       Motor 3 : Back (CCW)
%       Motor 4 : Right (CW)
%       +X : Front
%       +Y : Left
%       +Z : UP
%
%   Rotation Order - 'ZYX'
%   Quaternion notation : [qw qx qy qz]'
%
%Author : Eranga Fernando
%Email  : hctef2@mun.ca, hcteranga@gmail.com
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%     _   _   _   _   _   _     %%%
%%%    / \ / \ / \ / \ / \ / \    %%%
%%%   ( H | C | T | E | F | 2 )   %%%
%%%    \_/ \_/ \_/ \_/ \_/ \_/    %%%
%%%                               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef Quadrotor < handle
    properties
        %% Data storing for analysis
        
        id;                 % ID of the robot
        input;              %Input variables (Roll, Pitch, Yaw, Thrust)        
        groundtruth;        % p,p_dot,q,q_dot,v,v_dot,ba,w,w_dot,bg
        imu_gt;             % IMU Ground truth
        imu_m;              % IMU measurements with measurement noises        
        measurements_gt;    % Other measurements ground truth (Range, Attitue, Thrust Rate)        
        measurements_m;     % Other measurements with noise (Range, Attitue, Thrust Rate)
        %% Temporary variables
        % States
        
        p;                  % Position in the world frame [X Y Z]'
        p_dot;              % Velocity in world frame
        q;                  % Rotation from body frame to world frame [w x y z]'
        q_dot;              % Rotation rate
        v;                  % Velocity in body frame [Vx Vy Vz]'
        v_dot;              % Linear accelerarion in body frame
        w;                  % Angular velocity in body rame [wx wy wz]'
        w_dot;              % Angular acceleration in body frame
        rpy;                % Attitude
        h;                  % Height
        %% Sensor measurements
        
        bg;                 % Gyroscope bias [Bgx Bgy Bgz]'
        ba;                 % Accelerometer bias [Bax Bay Baz]'
        h_m;                % Height measurement
        acc_m;              % Acceleration measurements [ax ay az]'
        gyro_m;             % Gyroscope measurements [gx gy gz]'
        prop_spd_m;         % Propeller speed measurements [p1 p2 p3 p4]'
        rpy_m;              % Roll pitch and yaw values    
        
        % Others
        
        no_beacons;         % Number of range beacons
        beacon_loc;         % Beacon locations [X Y Z]'

        C;                  % Rotation matrix from body to world        
        e1 = [1 0 0]';      % Basis vectors
        e2 = [0 1 0]';      % Basis vectors
        e3 = [0 0 1]';      % Basis vectors
        
        %% Simulation Parameters
        
        noise;              % Add noise for the input of the simulation 
        % noise = 1 : Run the model with noise
        % noise = 0 : Run the model without noise
        bias_on             % Enabling the gyrocopic and accelerometer biases
        % bias_on = 1 : Add bias for measurements
        % bias_on = 0 : No bias in the measurements
        
        dt;                 % Simulation time
        max_iterations;     % Maximum number of iterations.
        % This allow fast execution of codes since memory is allocated
        % initially
        
        n;                  % Nth iteration 
        Pw;                 % Anchor positions
        %% Visualizing Parameters
        % Haven't implemented yet
        viz = 1;
        f_handle;
        
        % Kinematic simulator variables
        
        eul_dot;            % Euler angle rates
        
        % Dynamic simulator variables
        
        roll_error;         % Roll error
        pitch_error;        % Pitch error
        yaw_error;          % Yaw error
        out;                % Output of the quadrotor dynamic function. (States)
        %% System parameters
        
        param;              % Process Noise
        quad;               % physical parameters of the quadrotor

    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
       
        function this = Quadrotor(Id,X_int,param,dt,f_handle,noise,iterations)
        %% Construct
        % Inputs :
        % Id : Quadrotor ID
        % X_int : Initial states : [p q v w]' : v : body velocity
        % param : Structure containing parameters : Parameters are stored
        % in sys_parameters.m
        % dt : Simulation time
        % f_handle : Figure handles
        % noise : Enable dissable noise
        % iterations : Number of iterations
            
            % Initializing parameters
            this.id = Id;
            
            % Simulation parameters
            this.dt = dt;
            this.f_handle = f_handle;
            this.noise = noise;
            this.max_iterations = iterations;
            this.n = 0; %initialize the iterator to 0
            this.bias_on = 1; % Enabling bias
            
            % Range beacon locations
            this.no_beacons = param.no_beacons;
            this.beacon_loc = param.beacon_loc;
            
            % Data storing
            this.input = struct ('in',zeros(4,this.max_iterations),'U_dot',zeros(1,this.max_iterations));
            this.groundtruth = struct('p',zeros(3,this.max_iterations),'p_dot',zeros(3,this.max_iterations),'q',zeros(4,this.max_iterations),...
                'q_dot',zeros(4,this.max_iterations),'v',zeros(3,this.max_iterations),'v_dot',zeros(3,this.max_iterations),'ba',zeros(3,this.max_iterations),...
                'w',zeros(3,this.max_iterations),'w_dot',zeros(3,this.max_iterations),'bg',zeros(3,this.max_iterations));
            this.imu_m = struct('acc',zeros(3,this.max_iterations),'gyro',zeros(3,this.max_iterations));
            this.imu_gt = struct('acc',zeros(3,this.max_iterations),'gyro',zeros(3,this.max_iterations));
            this.measurements_gt = struct('prop_spd',zeros(4,this.max_iterations),'h',zeros(1,this.max_iterations),'atti',zeros(3,this.max_iterations),'ranges',zeros(this.no_beacons,this.max_iterations));
            this.measurements_m = struct('prop_spd',zeros(4,this.max_iterations),'h',zeros(1,this.max_iterations),'atti',zeros(3,this.max_iterations),'ranges',zeros(this.no_beacons,this.max_iterations));
            
            % Update initial values
            this.p = X_int(1:3)';
            this.q = X_int(4:7)';
            this.v = X_int(8:10)';
            this.w = X_int(11:13)';
            this.p_dot = zeros(3,1);
            this.q_dot = zeros(4,1);
            this.v_dot = zeros(3,1);
            this.w_dot = zeros(3,1);
            this.h = X_int(3);
            this.bg = zeros(3,1);
            this.ba = zeros(3,1);
            eul = quat2eul(this.q');
            this.rpy = [eul(3) eul(2) eul(1)]';
            
            
            % Initializing the dynamic model controller
            this.roll_error = 0;
            this.pitch_error = 0;
            this.yaw_error = 0;
            [yaw, pitch, roll] = quat2angle(this.q');
            this.rpy_m = [roll pitch yaw]';
            this.out = zeros(22,1);
            this.out(13:15) = this.p;
            this.out(7:9) = this.v;
            this.out(10:12) = this.w;
            this.out(16:18) = this.rpy_m;
            
            
            % Initialize rotation matrix
            this.C = quat2rotm(this.q');
            
            % Physical parameters
            quad = struct('m',0,'I',0,'invI',0,'g',0,'k',0,'k_pp',0,'k_pa',0,'b',0,'d',0,'l',0,'Eb',0,'invEb',0,'Ew',0,'prop_norm',0,'e1',[1 0 0]','e2',[0 1 0]','e3',[0 0 1]');
            this.quad.k       = param.k;    % Propeller thrust coefficient
            this.quad.k_pp    = param.k_pp; % Perpendicular drag coefficient
            this.quad.k_pa    = param.k_pa; % Parallel drag coefficient
            this.quad.m       = param.m;    % Mass of the quadrotor
            this.quad.g       = param.g;    % Gravitational acceleration (not a vector, g = 9.81)
            this.quad.b = param.b;          % Propeller thrust coefficient
            this.quad.d = param.d;          % Propeller drag coefficient
            this.quad.l = param.l;          % Distance from propeller to oposite propeller
            this.quad.Eb =  [[              this.quad.k,             this.quad.k,             this.quad.k,              this.quad.k];
                             [                        0, this.quad.k*this.quad.l,                       0, -this.quad.k*this.quad.l];
                             [ -this.quad.k*this.quad.l,                       0, this.quad.k*this.quad.l,                        0];
                             [             -this.quad.d,             this.quad.d,            -this.quad.d,              this.quad.d]];
            this.quad.invEb = inv(this.quad.Eb);
            this.quad.I = param.I;          % Moment of inertia
            this.quad.invI = inv(this.quad.I);
            this.quad.Ew = this.quad.Eb(2:4,:);
            this.quad.prop_norm = sqrt(this.quad.m*this.quad.g/(4*this.quad.k));  % Nominal speed of a single prop
            
            % Measurement noise parameters
            % https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
            % IMU parameters
            sqrtdt = sqrt(this.dt);
            this.param.sim_sigma_Ax = param.sim_sigma_Ax/sqrtdt;
            this.param.sim_sigma_Ay = param.sim_sigma_Ay/sqrtdt;
            this.param.sim_sigma_Az = param.sim_sigma_Az/sqrtdt;
            this.param.sim_sigma_Gx = param.sim_sigma_Gx/sqrtdt;
            this.param.sim_sigma_Gy = param.sim_sigma_Gy/sqrtdt;
            this.param.sim_sigma_Gz = param.sim_sigma_Gz/sqrtdt;
            this.param.sim_sigma_bg = param.sim_sigma_bg*sqrtdt;
            this.param.sim_sigma_ba = param.sim_sigma_ba*sqrtdt;
            
            % Other measurements
            this.param.sim_sigma_h = param.sim_sigma_h;
            this.param.sim_sigma_prop = param.sim_sigma_prop;
            this.param.sim_sigma_uwb = param.sim_sigma_uwb;
            this.param.sim_sigma_rpy = param.sim_sigma_rpy; % 3x1 matrix
            
            
            % Process model noise
            this.param.sigma_roll = param.sigma_roll;
            this.param.sigma_pitch = param.sigma_pitch;
            this.param.sigma_yaw = param.sigma_yaw;
            
            % Propeller parameters
            this.out(19:22) = ones(4,1)*this.quad.prop_norm;
        end
        
        function kinematic_sim(this,roll,pitch,yaw,thrust)
        %% Kinematic Simulator
        % This function runs a kinematic simulation of the quadrotor for a
        % given inputs
        % Inputs :
        % roll   : Roll control signal from the trajectory controller
        % pitch  : Pitch control signal from the trajectory controller
        % yaw    : Yaw control signal from the trajectory controller
        % Thrust : Thrust from the trajectory controller
            %% Simulation
            tau = 1/200;
            % Inrease the iterator
            this.n = this.n+1;
            % Store the inputs
            this.input.in(:,this.n) = [roll pitch yaw thrust]';
            eul = quat2eul(this.q');
            n_eul = 0.95*eul + 0.05*[yaw pitch roll];
            roll = n_eul(3); pitch = n_eul(2); yaw = n_eul(1);
            % Calculating the sum of propeller speeds % Assuming near
            % hovering case
            if this.n ~= 1
                omega_n_1 = 2*sqrt(this.input.in(4,this.n-1)/this.quad.k);
                omega = 2*sqrt(thrust/this.quad.k);
            else
                omega = 0;
                omega_n_1 = 4*this.quad.prop_norm;
            end
            % Enabling and dissableing process noise
            if(this.noise == 1)
                roll = roll + this.param.sigma_roll*randn(1);
                pitch = pitch + this.param.sigma_pitch*randn(1);
                yaw = yaw + this.param.sigma_yaw*randn(1);
            end
            % Calculating angular rates
            if(this.n < 3 ) % Check this condition
                eul = quat2eul(this.q');
                troll = eul(3); tpitch = eul(2); tyaw = eul(1);
                if (abs(tyaw) > 3 && (tyaw > 0 && yaw < 0))
                    disp('increasing')
                    yaw = 2*pi + yaw;
                elseif (abs(tyaw) > 3 && (tyaw < 0 && yaw > 0))
                    disp('decreasing')
                    yaw = -2*pi + yaw;
                end                      
                this.eul_dot = ([roll, pitch, yaw]' - [troll, tpitch, tyaw]')/this.dt;
                this.rpy = this.rpy + this.eul_dot*this.dt;
                Tinv = [ [ 1,           0,           -sin(tpitch)]
                    [ 0,  cos(troll), cos(tpitch)*sin(troll)]
                    [ 0, -sin(troll), cos(troll)*cos(tpitch)]];
            else
                eul = quat2eul(this.q');
                troll = eul(3); tpitch = eul(2); tyaw = eul(1);
                eul = quat2eul(this.groundtruth.q(:,this.n-2)');
                %ttroll = eul(3); ttpitch = eul(2); ttyaw = eul(1);
                if (abs(tyaw) > 3 && (tyaw > 0 && yaw < 0))
                    yaw = 2*pi + yaw;
                elseif (abs(tyaw) > 3 && (tyaw < 0 && yaw > 0))
                    yaw = -2*pi + yaw;
                end
                
                this.eul_dot = (1-this.dt/tau)*this.eul_dot + (this.dt/tau)*([roll, pitch, yaw]'-[troll, tpitch, tyaw]')/(this.dt);
                Tinv = [ [ 1,           0,           -sin(tpitch)]
                    [ 0,  cos(troll), cos(tpitch)*sin(troll)]
                    [ 0, -sin(troll), cos(troll)*cos(tpitch)]];
                this.rpy = this.rpy + this.eul_dot*this.dt;
            end
            this.rpy(3) = wrapToPi(this.rpy(3));
            w_temp = Tinv*this.eul_dot;
            
            % Angular accelerations in body frame
            % this.w_dot = (1 - this.dt/tau)*this.w_dot + (this.dt/tau)*(w_temp-this.w)/this.dt;
            this.w_dot = (w_temp-this.w)/this.dt;
            % Linear accelerations in body frame
            
            if this.n~= 1
                this.v_dot =skew_mat(this.v)*this.w - this.C'*this.e3*this.quad.g - diag([this.quad.k_pp, this.quad.k_pp, this.quad.k_pa])*this.v*omega_n_1/this.quad.m + this.input.in(4,this.n-1)*this.e3/this.quad.m;
            else
                this.v_dot = skew_mat(this.v)*this.w - this.C'*this.e3*this.quad.g - diag([this.quad.k_pp, this.quad.k_pp, this.quad.k_pa])*this.v*omega_n_1/this.quad.m + this.quad.g*this.e3;
            end
            % Quaternion rate
            this.q_dot = 0.5*quatmultiply(this.q',[0 this.w'])';
            % Update angular velocity
            this.w = this.w + this.w_dot*this.dt;
            % Update linear velocity
            this.v = this.v + this.v_dot*this.dt;
            
            % Orientation
            q1 = this.q + this.q_dot*this.dt;
            %% -- TO DO --
            % Have to check for the direction of the quaternion when Euler
            % angle is wraped to PI. Otherwise working fine.
            this.q = eul2quat([this.rpy(3),this.rpy(2),this.rpy(1)])';
            %% -----------
            this.C = quat2rotm(this.q');
                        
            %World frame quantities
            this.p_dot = this.C*this.v;
            if this.n ~= 1
                this.p = this.p + this.groundtruth.p_dot(:,this.n-1)*this.dt;
            end

            % Calculate range to no_beacon
            r_vec = repmat(this.p,1,this.no_beacons) - this.beacon_loc;
            ranges = zeros(this.no_beacons,1);
            for i =1:this.no_beacons
                ranges(i) = norm(r_vec(:,i));
            end
            
            %% Measurements of n'th time step
            % IMU measurements
            % Groundtruth            
            this.acc_m = -diag([this.quad.k_pp, this.quad.k_pp, this.quad.k_pa])*this.v*omega/this.quad.m + this.e3*thrust/this.quad.m;
            this.imu_gt.gyro(:,this.n) = this.w;
            this.imu_gt.acc(:,this.n) = this.acc_m;
            this.measurements_gt.prop_spd(:,this.n) = repmat(omega/4,1,4);
            this.measurements_gt.h(:,this.n) = this.p(3);
            this.measurements_gt.atti(:,this.n) = this.rpy;
            this.measurements_gt.ranges(:,this.n) = ranges;
            % Measurements with noise
            this.bg = this.bg + this.param.sim_sigma_bg*this.dt*randn(3,1);
            this.ba = this.ba + this.param.sim_sigma_ba*this.dt*randn(3,1);
            this.imu_m.gyro(:,this.n) = this.w + diag([this.param.sim_sigma_Gx, this.param.sim_sigma_Gy, this.param.sim_sigma_Gz])*randn(3,1) + this.bias_on*this.bg;
            this.imu_m.acc(:,this.n) = this.acc_m + diag([this.param.sim_sigma_Ax, this.param.sim_sigma_Ay, this.param.sim_sigma_Az])*randn(3,1) + this.bias_on*this.ba;
            this.measurements_m.prop_spd(:,this.n) = repmat(omega/4,4,1) + this.param.sim_sigma_prop* randn(4,1);
            this.measurements_m.h(:,this.n) = this.p(3) + this.param.sim_sigma_h*randn(1);
            this.measurements_m.atti(:,this.n) = this.rpy + diag(this.param.sim_sigma_rpy)*randn(3,1);
            this.measurements_m.ranges(:,this.n) = ranges + this.param.sim_sigma_uwb*randn(this.no_beacons,1);
            % Groundtruth
            if(this.n ~= 1)
                this.groundtruth.p(:,this.n) = this.p;
                this.groundtruth.p_dot(:,this.n) = this.p_dot;
                this.groundtruth.q(:,this.n) = this.q;
                this.groundtruth.q_dot(:,this.n) = this.q_dot;
                this.groundtruth.v(:,this.n) = this.v;
                this.groundtruth.v_dot(:,this.n-1) = this.v_dot;
                this.groundtruth.w(:,this.n) = this.w;
                this.groundtruth.w_dot(:,this.n-1) = this.w_dot;
                this.groundtruth.bg(:,this.n) = this.bg;
                this.groundtruth.ba(:,this.n) = this.ba;
                this.input.U_dot(1,this.n) = (thrust - this.input.in(4,this.n-1))/this.dt; 
            else
                this.groundtruth.p(:,this.n) = this.p;
                this.groundtruth.p_dot(:,this.n) = this.p_dot;
                this.groundtruth.q(:,this.n) = this.q;
                this.groundtruth.q_dot(:,this.n) = this.q_dot;
                this.groundtruth.v(:,this.n) = this.v;
                this.groundtruth.w(:,this.n) = this.w;
                this.groundtruth.bg(:,this.n) = this.bg;
                this.groundtruth.ba(:,this.n) = this.ba;
            end
            
        end
        
        
        function this = dynamic_sim(this,roll,pitch,thrust)
        %% Dynamic Simulator 
        %     
            %%-- TO DO --%%
        end
        
        
        function visualize(this)
        %% Visualize the results
            time = (1:this.n)*this.dt;
            figure;
            plot3(this.groundtruth.p(1,:),this.groundtruth.p(2,:),this.groundtruth.p(3,:));
            figure;
            plot(time,this.groundtruth.p(1,:),time,this.groundtruth.p(2,:),time,this.groundtruth.p(3,:));            
            title('Groundtruth Position');
            figure;
            plot(time,this.groundtruth.p_dot(1,:),time,this.groundtruth.p_dot(2,:),time,this.groundtruth.p_dot(3,:));
            title('Groundtruth World Velocity');
            rpy = quat2eul(this.groundtruth.q');
            figure;
            plot(time',rpy(:,3),time',rpy(:,2),time',rpy(:,1))
            title('Groundtruth attitude');
            figure;
            plot(time,this.groundtruth.v(1,:),time,this.groundtruth.v(2,:),time,this.groundtruth.v(3,:));
            title('Groundtruth body velocity');
            figure;
            plot(time,this.groundtruth.v_dot(1,:),time,this.groundtruth.v_dot(2,:),time,this.groundtruth.v_dot(3,:));
            title('Groundtruth body acceleration');
            figure;
            plot(time,this.groundtruth.ba(1,:),time,this.groundtruth.ba(2,:),time,this.groundtruth.ba(3,:));
            title('Groundtruth accelerometer bias');
            figure;
            plot(time,this.groundtruth.w(1,:),time,this.groundtruth.w(2,:),time,this.groundtruth.w(3,:));
            title('Groundtruth body angular velocity');
            figure;
            plot(time,this.groundtruth.bg(1,:),time,this.groundtruth.bg(2,:),time,this.groundtruth.bg(3,:));
            title('Groundtruth Gyroscope Bias');
            figure;
            plot(time,this.measurements_gt.atti(1,:),time,this.measurements_gt.atti(2,:),time,this.measurements_gt.atti(3,:));
            title('Groundtruth Attitude');
            figure;
            plot(time,this.imu_gt.acc(1,:),time,this.imu_gt.acc(2,:),time,this.imu_gt.acc(3,:));
            title('Groundtruth IMU acceleration');
            figure;
            plot(time,this.imu_gt.gyro(1,:),time,this.imu_gt.gyro(2,:),time,this.imu_gt.gyro(3,:));
            title('Groundtruth IMU gyroscope');
            figure;
            plot(time,this.measurements_gt.ranges(1,:),time,this.measurements_gt.ranges(2,:),time,this.measurements_gt.ranges(3,:));
            title('Groundtruth Range measurements');
            
        end
    end
end