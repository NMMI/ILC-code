%% Initial cleaning
clc
close all
clear all
rng('shuffle');
warning off % All warnings are disabled

%% global variables
global N t_fin pre_time post_time T_sample samples selector Amp_step step_period...
       Kpon Kdon Kpoff Kdoff  x_0 dx_0 preset FF u_err_resampled TT_FF0 step_time... 
       u_int ref dref ddref r dr ddr preset_inertia s_t  control...
       tot_iter folder err_tot cont_tot err_ev cont_ev ind_0 ind_f i_0 i_f...
       err_rif err_rif_app gerr gerr_rif time  err derr qb_input u_plot q_plot r_plot   
 
%% Generic Initialization

% All the parameters files are in ./Parameters
% All the function scripts are in ./Functions
% All the simulink schemes are in ./Simulink
addpath('Parameters', 'Functions', 'Simulink');

cube_parameters;    % Parameters from qbmoves maker pro datasheet
model_parameters;   % Number of joints of the system and other constants used in the  simulink schemes
time_parameters;    % Definition of all time interval, sample time and number of samples

tot_iter = 0;       % It will store the total number of iterations
folder = create_new_folder(); % path of the directory to store the data

err_tot = 0;        % It will store the total error for each the iteration
cont_tot = 0;       % It will store the total control for each the iteration a

t_fin = ceil(t_fin_init/T_sample_init)*T_sample_init;           % Time interval to track the trajectory
pre_time = ceil(pre_time_init/T_sample_init)*T_sample_init;     % Time interval to place the robot in x_0
post_time = ceil(post_time_init/T_sample_init)*T_sample_init;	% Time interval to reposition the robot
step_time = ceil(step_time_init/T_sample_init)*T_sample_init;   % Used in the identification phase

if (t_fin - t_fin_init) ~= 0
    disp(strcat('The time interval t_fin has been increased of ',10, num2str(t_fin - t_fin_init)))
end
if (pre_time - pre_time_init) ~= 0
    disp(strcat('The time interval pre_time has been increased of ',10, num2str(pre_time - pre_time_init)))
end
if (post_time - post_time_init) ~= 0
    disp(strcat('The time interval post_time has been increased of ',10, num2str(post_time - post_time_init)))
end
if (step_time - step_time_init) ~= 0
    disp(strcat('The time interval step_time has been increased of ',10, num2str(step_time - step_time_init)))
end

T_sample = T_sample_init;   % Sample time
samples  = samples_init;    % Number of samples

%% Reference Trajectory

input_parameters;           % Preset and reference data

% Check if ref.time(end) == t_fin and resample ref
if ref.time(end) < t_fin
    r = timeseries(vertcat(deg2rad(ref.data),deg2rad(ref.data(end,:))) , vertcat(dref.time, t_fin));            % Reference
    r = resample(r, r.time(1) : T_sample : r.time(end));         

    dr = timeseries(vertcat(deg2rad(dref.data),deg2rad(dref.data(end,:))) , vertcat(dref.time, t_fin));        % Derivative of the reference
    dr = resample(dr, dr.time(1) : T_sample : dr.time(end));    

    ddr = timeseries(vertcat(deg2rad(ddref.data),deg2rad(ddref.data(end,:))) , vertcat(ddref.time, t_fin));     % Second derivative of the reference
    ddr = resample(ddr, ddr.time(1) : T_sample : ddr.time(end)); 
else
    r = timeseries(deg2rad(ref.data), ref.time);            % Reference
    r = resample(r, r.time(1) : T_sample : r.time(end));         

    dr = timeseries(deg2rad(dref.data), dref.time);         % Derivative of the reference
    dr = resample(dr, dr.time(1) : T_sample : dr.time(end));    

    ddr = timeseries(deg2rad(ddref.data), ddref.time);      % Second derivative of the reference
    ddr = resample(ddr, ddr.time(1) : T_sample : ddr.time(end)); 
end

x_0 = r.data(1,:);      % Reference initial position
dx_0 = dr.data(1,:);	% Derivative of the reference initial position

% Preset evolution (It may vary online, but it should be equal in each iterations)
if preset_init.time(end) < (pre_time + t_fin + post_time)
    preset = timeseries(vertcat(preset_init.data,preset_init.data(end,:)) , vertcat(preset_init.time, (pre_time + t_fin + post_time)));
    preset = resample(preset, preset.time(1) : T_sample : preset.time(end));         
else
    preset = preset_init;          
    preset = resample(preset, preset.time(1) : T_sample : preset.time(end));       
end

%% Identification phase

% Evaluate the resting position of the robot
sim('Evaluate_resting_position');
pause on;
pause(2);
pause off;
x_start = start_pos.data(end,:);
clear start_pos;

% State-space filter definition (used in 'Evaluate_positioning_control_action.slx')
[A_f,B_f,C_f,D_f] = tf2ss([1],[1 1]);

[size_A_f,~] = size(A_f);
filter_initial_condition = zeros(size_A_f*N,1);
for i = 1:N
    filter_initial_condition(i*size_A_f) = x_start(i)/C_f(end);
end

% Evaluate the control action to move the robot from the resting position
% to x_0 (reference initial position)
sim('Evaluate_positioning_control_action');

pause on;
pause(2);
pause off;

% clear A_f B_f C_f D_f size_A_f

% Feedforward control action to move the robot from the resting position
% to x_0 (reference initial position)
if u_err_0.time(end) < pre_time 
    u_err_resampled = timeseries(vertcat(u_err_0.signals.values, u_err_0.signals.values(end,:)) , vertcat(u_err_0.time, pre_time));
    u_err_resampled = resample(u_err_resampled, u_err_resampled.time(1) : T_sample : u_err_resampled.time(end));         
else
    u_err_resampled = timeseries(u_err_0.signals.values, u_err_0.time);
    u_err_resampled = resample(u_err_resampled, u_err_resampled.time(1): T_sample : u_err_resampled.time(end));     
end

J = zeros(N,1);
b = zeros(N,1);
kap = zeros(N,1);

% Evaluatate inertia and damping seen from each joint
[J, b, kap] = calc_inertia_and_damping(u_err_0.signals.values(end, :));


%% FF_0 evaluatation (Initial Guess)

TT_FF0 = zeros(length(r.time),N);
U = zeros(N, 1);
fun = @(f_eq_x, f_eq_I, f_eq_b, f_eq_r, f_eq_dr, f_eq_ddr, f_eq_D) f_eq(f_eq_x, f_eq_I, f_eq_b, f_eq_r, f_eq_dr, f_eq_ddr, f_eq_D);

% Evaluate control in t = 0 without Delta
 for i = 1:N
    f_eq_I = J(i);
    f_eq_b = b(i);
    f_eq_r = (r.data(1,i) - x_0(i));
    f_eq_dr = (dr.data(1,i)  - dx_0(i));
    f_eq_ddr = ddr.data(1,i);
    f_eq_D = preset.data((pre_time/T_sample + 1), i);
    
    fun_evaluated = @(f_eq_x) fun(f_eq_x, f_eq_I, f_eq_b, f_eq_r, f_eq_dr, f_eq_ddr, f_eq_D);
	U(i) = fzero(fun_evaluated, 0);  
 end

for i = 1:N
	U(i) = U(i) + u_err_0.signals.values(end, i);
end

Delta = zeros(2, N);

% Evaluate Delta disturbance and uncertainty
for i = 1:N
    in = [J(i); b(i); (r.data(1,i) - x_0(i)); (dr.data(1,i)  - dx_0(i)); U(i); preset.data((pre_time/T_sample + 1), i)];
    Delta(:, i) = calc_Delta(in);
end

% Evaluate feedforward control action FF_0
fun2 = @(f_eq2_x, f_eq2_I, f_eq2_b, f_eq2_r, f_eq2_dr, f_eq2_ddr, f_eq2_D, f_eq2_Delta) f_eq2(f_eq2_x, f_eq2_I, f_eq2_b, f_eq2_r, f_eq2_dr, f_eq2_ddr, f_eq2_D, f_eq2_Delta);
for i = 1 : samples
    for j = 1:N
        f_eq2_I = J(j);
        f_eq2_b = b(j);
        f_eq2_r = (r.data(i,j) - x_0(j));
        f_eq2_dr = (dr.data(i,j)  - dx_0(j));
        f_eq2_ddr = ddr.data(i,j);
        f_eq2_D = preset.data((pre_time/T_sample + i), j);
        f_eq2_Delta = Delta(2, j);  

        fun2_evaluated = @(f_eq2_x) fun2(f_eq2_x, f_eq2_I, f_eq2_b, f_eq2_r, f_eq2_dr, f_eq2_ddr, f_eq2_D, f_eq2_Delta);

        TT_FF0(i,j) = fzero(fun2_evaluated, 0);
    end
end

% Add pre_time and post_time to FF_0

FF.time = [0:T_sample:(t_fin + pre_time + post_time)]';

FF.signals.values = zeros(length(FF.time),N);   
FF.signals.dimensions =  N;

FF.signals.values((pre_time/T_sample + 1) : (pre_time + t_fin)/T_sample + 1, :) = TT_FF0;

for i = ((pre_time + t_fin)/T_sample + 2):length(FF.signals.values)
    FF.signals.values(i, :) = TT_FF0(end,:);
end


for i = 1:N
    FF.signals.values(1 : (pre_time/T_sample), i) = u_err_resampled.data(1:(pre_time/T_sample) , i);
end

clear U fun_evaluated fun2_evaluated in f_eq_I f_eq_b f_eq_r f_eq_dr f_eq_ddr f_eq_D f_eq2_I f_eq2_b f_eq2_r f_eq2_dr f_eq2_ddr f_eq2_D f_eq2_Delta
%% Linearized decoupled model (state = q1 dq1 q2 dq2....)

X = zeros(2,2,N);
Y = zeros(2,1,N);

A = zeros(N*2, N*2, samples);	% State matrix
B = zeros(N*2, N, samples);     % Input matrix

for i = 1 : samples
	for j = 1:N
    	in = [J(j); b(j); r.data(i,j); dr.data(i,j); TT_FF0(i,j); preset.data((pre_time/T_sample + i), j); Delta(1); Delta(2)];
        [X(:,:,j), Y(:,:,j)] = cube_linearization(in);
        if j == 1
           App1 = X(:,:,j);
           App2 = Y(:,:,j);
        else
           App1 = blkdiag(App1, X(:,:,j));
           App2 = blkdiag(App2, Y(:,:,j));
        end
    end
    A(:,:,i) = App1;
    B(:,:,i) = App2;
end

clear X Y App1 App2 in
%% ILC parameters evaluation

ILC_parameters;

%% Kon

Kpon.time = [0:T_sample:(t_fin + pre_time + post_time)]';
Kdon.time = [0:T_sample:(t_fin + pre_time + post_time)]';
Kpon.signals.values = zeros(length(Kpon.time),N);   
Kdon.signals.values = zeros(length(Kdon.time),N);  
Kpon.signals.dimensions =  N;
Kdon.signals.dimensions =  N;


% Evaluate Kon through TVLQR

Sr0 = [0; 0; 0; 0];

time_S = [0:T_sample:t_fin];

p_appo = zeros(length(Kpon.time),N);
d_appo = zeros(length(Kpon.time),N);

for i =1:N
    % Riccati equation
    [T Sr] = ode45(@(t,Sr)myRiccati(t, Sr, A(2*i-1:2*i, 2*i-1:2*i, :), B(2*i-1:2*i, i, :), diag([1 0]), R, time_S), fliplr(time_S), Sr0);
   

    [m n] = size(Sr);
    S = mat2cell(Sr, ones(m,1), n);
    fh_reshape = @(x)reshape(x,size(squeeze(A(1:2,1:2,1))));
    S = cellfun(fh_reshape,S,'UniformOutput',false);

    SS_appo = zeros(2,2,samples);
    
    for j = 1 : samples
       SS_appo(:,:, (samples + 1 -j)) = S{j}; 
    end

    S = SS_appo;
    for j = 1:samples
        retro = R^-1*B(2*i-1:2*i, i, j)'*S(:,:,j);     % TVLQR solution
        p_appo((pre_time/T_sample + j), i) = retro(1);
        d_appo((pre_time/T_sample + j), i) = retro(2);
    end
end
for i = 1:N
    if p_appo(((pre_time + t_fin)/T_sample + 1), i) == 0 
        p_appo(((pre_time + t_fin)/T_sample + 1), i) =  p_appo(((pre_time + t_fin)/T_sample), i);
    end
    if d_appo(((pre_time + t_fin)/T_sample + 1), i) == 0
        d_appo(((pre_time + t_fin)/T_sample + 1), i) =  d_appo(((pre_time + t_fin)/T_sample), i);
    end
end

delay = 25;
p_ff_appo = zeros(length((pre_time/T_sample + 1) : ((pre_time + t_fin)/T_sample + 1)) + 2*delay, N);
d_ff_appo = zeros(length((pre_time/T_sample + 1) : ((pre_time + t_fin)/T_sample + 1)) + 2*delay, N);

p_appo_filtered = zeros(length((pre_time/T_sample + 1) : ((pre_time + t_fin)/T_sample + 1)) + 2*delay, N);
d_appo_filtered = zeros(length((pre_time/T_sample + 1) : ((pre_time + t_fin)/T_sample + 1)) + 2*delay, N);

[num_f,den_f] = butter(2,0.1);


for i=1:N
    p_ff_appo(:,i) = vertcat(   ones(delay,1)*p_appo((pre_time/T_sample + 1),i),...
                                p_appo((pre_time/T_sample + 1) : (pre_time + t_fin)/T_sample + 1, i),...
                                ones(delay,1)*p_appo(((pre_time + t_fin)/T_sample + 1),i));
                            
    d_ff_appo(:,i) = vertcat(	ones(delay,1)*d_appo((pre_time/T_sample + 1),i),...
                                d_appo((pre_time/T_sample + 1) : (pre_time + t_fin)/T_sample + 1, i),...
                                ones(delay,1)*d_appo(((pre_time + t_fin)/T_sample + 1),i));                          
end


p_appo_filtered = filtfilt(num_f,den_f,p_ff_appo);
d_appo_filtered = filtfilt(num_f,den_f,d_ff_appo);


Kpon.signals.values((pre_time/T_sample + 1) : (pre_time + t_fin)/T_sample + 1, :) = p_appo_filtered((delay+1):end-delay,:); 
Kdon.signals.values((pre_time/T_sample + 1) : (pre_time + t_fin)/T_sample + 1, :) = d_appo_filtered((delay+1):end-delay,:); 

% Add pre_time and post_time
for i = 1:N
    Kpon.signals.values(1 : (pre_time/(2*T_sample) + 1), i) = (Kpon.signals.values(pre_time/T_sample + 1, i))/10;
    Kpon.signals.values((pre_time + t_fin)/T_sample + 2 : end, i) = Kpon.signals.values((pre_time + t_fin)/T_sample + 1, i)/5;
    Kdon.signals.values(1 : (pre_time/(2*T_sample) + 1), i) = (Kdon.signals.values(pre_time/T_sample + 1, i))/10; 
end

step_p = ((Kpon.signals.values((pre_time/T_sample + 1), :) - Kpon.signals.values(1, :))/(pre_time/(2*T_sample)));
step_d = ((Kdon.signals.values((pre_time/T_sample + 1), :) - Kdon.signals.values(1, :))/(pre_time/(2*T_sample)));
for i =  (pre_time/(2*T_sample) + 2) : (pre_time/T_sample)
    Kpon.signals.values(i, :) = (Kpon.signals.values(i - 1, :) + step_p);
    Kdon.signals.values(i, :) = (Kdon.signals.values(i - 1, :) + step_d);
end

clear step_p step_d p_appo_filtered d_appo_filtered p_ff_appo d_ff_appo p_appo d_appo SS_appo retro Sr m n fh_reshape T
%% Koff

% Evaluate Kdoff
Kpoff = zeros(samples, N);
Kdoff = zeros(samples, N);
for i = 1:samples
    [Kpoff(i, :), Kdoff(i, :)] = calc_Koff(B(:,:,i));
end

Kdoff = (1+epsilon)*Kdoff; 

AVG_on = zeros(2,1);

% Evaluate Kpoff
for i = 1:N
    AVG_on(i) = mean(Kpon.signals.values((pre_time/T_sample + 1) : (pre_time + t_fin)/T_sample + 1, i))/mean(Kdon.signals.values((pre_time/T_sample + 1) : (pre_time + t_fin)/T_sample + 1, i));
    Kpoff(:,i) = AVG_on(i)* Kdoff(:,i);
end

for i = 1:N
    Kpoff(:,i) = mean(Kpoff(:,i))
    Kdoff(:,i) = mean(Kdoff(:,i))
    Kpon.signals.values((pre_time/T_sample + 1) : (pre_time + t_fin)/T_sample + 1, i) = mean(Kpon.signals.values((pre_time/T_sample + 1) : (pre_time + t_fin)/T_sample + 1, i));
    Kdon.signals.values((pre_time/T_sample + 1) : (pre_time + t_fin)/T_sample + 1, i) = mean(Kdon.signals.values((pre_time/T_sample + 1) : (pre_time + t_fin)/T_sample + 1, i));

end

%% Store all parameters in folder
store_parameters;

