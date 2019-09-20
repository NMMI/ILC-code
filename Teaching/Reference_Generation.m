%% Initial cleaning
clc
close all
clear all
rng('shuffle');
warning off % All warnings are disabled

%% global variables
global N t_fin T_sample samples ref dref ddref
%% Generic Initialization

cd ..
% All the parameters files are in ./Parameters
% All the function scripts are in ./Functions
% All the simulink schemes are in ./Simulink
addpath('Parameters', 'Functions', 'Simulink');

model_parameters;   % Number of joints of the system and other constants used in the  simulink schemes
time_parameters;    % Definition of all time interval, sample time and number of samples

cd Teaching

t_fin = ceil(t_fin_init/T_sample_init)*T_sample_init;          % Time interval to track the trajectory
T_sample = T_sample_init;   % Sample time

samples  = samples_init;    % Number of samples

%% Trajectory Teaching

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cut_time = 2; % Time interval at the beginning and at the end of the trajectory
cut_time = ceil(cut_time/T_sample_init)*T_sample_init; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

check = 'no';
while(~is_ok(check)) % Repeat the teaching unitl you are satisfied
    prompt = 'Are you ready to start?(y/n)';
    check = input(prompt, 's');
    if (is_ok(check))
        sim('Trajectory_Teaching.slx');
        pause on;
        pause(2);
        pause off;
        prompt = 'Is the reference ok?(y/n)';
        check = input(prompt, 's');
    end
end

%% Generate Reference, Derivative of the Reference and Second Derivative of the Reference
% Cut off the first and last cut_time seconds
i_0 = find(ref.time >= cut_time, 1);
i_0 = i_0 - 1;
i_f = find(ref.time >= (cut_time + t_fin), 1);

period = 0.0001;
tolerance = period - period*0.1;
time = ref.time(i_0 : i_f);
ref_appo = timeseries(ref.data((i_0:i_f), :), time);
ref_appo = resample(ref_appo, time(1): period : time(end));

ind_0 = find_index(ref_appo.time, cut_time, tolerance, 1);
ind_f = find_index(ref_appo.time, (t_fin + cut_time), tolerance, -1);

ref = 0;

ref = timeseries(ref_appo.data(ind_0 : ind_f, :), (ref_appo.time(ind_0 : ind_f) - cut_time));
ref = resample(ref, ref.time(1): T_sample : ref.time(end));

% Reference filtering
delay = 25;

ref_appo = zeros(length(ref.data) + 2*delay, N);

for i=1:N
   ref_appo(:,i) = vertcat( ones(delay,1)*ref.data(1,i),...
                            ref.data(:,i),...
                            ones(delay,1)*ref.data(end,i)); 
end

ref_appo_filtered = zeros(length(ref_appo), N);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[num_f,den_f] = butter(2,0.03);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ref_appo_filtered = filtfilt(num_f,den_f,ref_appo);

ref = timeseries(rad2deg(ref_appo_filtered((delay+1):end-delay,:)), ref.time);   % Reference trajectory  

% Evalutate derivative of the reference
for i = 1:N
     dref_appo(:, i) = gradient(ref.data(:, i))/T_sample; 
end

dref = timeseries(dref_appo, ref.time); % derivative of the reference trajectory  

% Evalutate second derivative of the reference
for i = 1:N
     ddref_appo(:, i) = gradient(dref.data(:, i))/T_sample; 
end

ddref = timeseries(ddref_appo, ref.time); % second derivative of the reference trajectory  

%% Save ref, dref and ddref in ./Parameters
cd ..
cd Parameters

save('reference', 'ref', 'dref', 'ddref')

cd ..
cd Teaching

%% Plot
figure;
plot(ref.time, ref.data);
grid on;
title('Reference');

figure;
plot(dref.time, dref.data);
grid on;
title('Derivative of the Reference');

figure;
plot(ddref.time, ddref.data);
grid on;
title('Second Derivative of the Reference');
