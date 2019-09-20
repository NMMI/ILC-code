%% input_parameters
% Used to store the preset profile and any parameters used to define the
% reference trajectory

global t_fin N T_sample samples

% reference trajectory
load('reference');

% preset profile
preset_init = timeseries(zeros(length([0:T_sample:(t_fin + pre_time + post_time)]'),N),[0:T_sample:(t_fin + pre_time + post_time)]');

%% Define here preset_init.data