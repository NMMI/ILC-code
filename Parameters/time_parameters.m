%% time_parameters

global N

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t_fin_init =        12;	% Trajectory Time Interval
pre_time_init =     12; % Time interval to place the robot in the initial configuration x_0
post_time_init =	6;	% Time interval to reposition the robot in a safe position after the trajectory execution

step_time_init = 3*ones(1,N); % Used in the identification phase
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

T_sample_init = 0.025;  % Sample_time, t_fin_init pre_time_init and 
                        % post_time_init should be multiples of T_sample

samples_init  = ceil(t_fin_init/T_sample_init) + 1; % Number of samples
