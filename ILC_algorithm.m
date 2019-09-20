function [FF_out] = ILC_algorithm(max_iter, reset, error_threshold)
%% This function performs ILC algorithm 
% max_iter = number of iterations
% reset = 0: continue the previous learning process
% reset = 1: create a new data folder and start a new learning process
% error_threshold = 
    %% global variables
    global N t_fin pre_time post_time T_sample samples Amp_step step_period...
           Kpon Kdon Kpoff Kdoff  x_0 dx_0 preset FF u_err_resampled TT_FF0 step_time... 
           u_int ref dref ddref r dr ddr preset_inertia s_t control...
           tot_iter folder err_tot cont_tot err_ev cont_ev ind_0 ind_f i_0 i_f...
           err_rif err_rif_app gerr gerr_rif time  err derr qb_input u_plot q_plot r_plot   

    %% Algorithm initialization, if reset == 1 the learning phase is resetted
    
    % All the parameters files are in ./Parameters
    % All the function scripts are in ./Functions
    % All the simulink schemes are in ./Simulink
    addpath('Parameters', 'Functions', 'Simulink');
    
    if nargin < 3
        error_threshold = 0;
    end
    
    if reset == 1 
        tot_iter = 0;
        err_tot = 0;
        cont_tot = 0;
        folder = create_new_folder();   % a new folder is created
        store_parameters;               % and trial parameters are stored

        control = zeros(samples, N);
        FF.time = [0:T_sample:(t_fin + pre_time + post_time)]';
        FF.signals.values = zeros(length(FF.time),N);
        FF.signals.values((pre_time/T_sample + 1) : (pre_time + t_fin)/T_sample + 1, :) = TT_FF0;
        for i = 1:N
            FF.signals.values(1 : (pre_time/T_sample), i) = u_err_resampled.data(1:(pre_time/T_sample) , i);
        end
        for i = ((pre_time + t_fin)/T_sample + 2):length(FF.signals.values)
            FF.signals.values(i, :) = TT_FF0(end,:);
        end
        control = FF.signals.values((pre_time/T_sample + 1) : (pre_time + t_fin)/T_sample + 1, :);
        FF.signals.dimensions = N;
    else  
        if tot_iter ==  0 
            control = zeros(samples, N);
            control = FF.signals.values((pre_time/T_sample + 1) : (pre_time + t_fin)/T_sample + 1, :);
        end

    end

    err_ev = zeros(N, max_iter);	% It will store the error evolution over iterations
    cont_ev = zeros(N, max_iter);   % It will store the control evolution over iterations

    % Control parameters local copy
    Kpoff_l = Kpoff;
    Kdoff_l = Kdoff;
    Kpon_l  = Kpon.signals.values((pre_time/T_sample + 1) : (pre_time + t_fin)/T_sample  + 1, :);
    Kdon_l  = Kdon.signals.values((pre_time/T_sample + 1) : (pre_time + t_fin)/T_sample  + 1, :);
   
    %% ILC algotihm
    for iteration = 1:max_iter
        
        % print the number of the current iteration
        disp(strcat(10, 'Executing iteration number', 9, num2str(tot_iter + iteration)));

        sim('Trajectory_tracking');

        pause on;
        pause(2);
        pause off;


        % Select the correct time interval
        i_0 = find(err.time >= pre_time, 1);
        i_0 = i_0 - 1;
        i_f = find(err.time >= (pre_time + t_fin), 1);


        period = 0.0001;
        tolerance = period - period*0.1;
        time = err.time(i_0 : i_f);
        err_rif_app = timeseries(err.signals.values((i_0:i_f), :), time);
        err_rif_app = resample(err_rif_app, time(1): period : time(end));

        ind_0 = find_index(err_rif_app.time, pre_time, tolerance, 1);
        ind_f = find_index(err_rif_app.time, (pre_time + t_fin), tolerance, -1);

        err_rif = timeseries(err_rif_app.data(ind_0 : ind_f, :), err_rif_app.time(ind_0 : ind_f));
        err_rif = resample(err_rif, err_rif.time(1): T_sample : err_rif.time(end));

        
        % Error filtering
        delay = 25;
       
        err_appo = zeros(length(err_rif.data) + 2*delay, N);
        
        for i=1:N
           err_appo(:,i) = vertcat( ones(delay,1)*err_rif.data(1,i),...
                                    err_rif.data(:,i),...
                                    ones(delay,1)*err_rif.data(end,i)); 
        end
     
        err_appo_filtered = zeros(length(err_appo), N);
        [num_f,den_f] = butter(2,0.1);
        err_appo_filtered = filtfilt(num_f,den_f,err_appo);


        err_rif = timeseries(err_appo_filtered((delay+1):end-delay,:), err_rif.time);   % Trajectory Tracking error          

        % Evalutate derivative of the error
        for i = 1:N
             gerr(:, i) = gradient(err_rif.data(:, i))/T_sample; 
        end

        gerr_rif = timeseries(gerr, err_rif.time); % derivative of the TT error 


        % Feedforward control is updated using calculated errors (offline-ILC)
        control = control + Kpoff_l.*err_rif.data + Kdoff_l.*gerr_rif.data;

        % Feedforward control is updated using feedback control signals (online-ILC)
        control = control + Kpon_l.*err_rif.data + Kdon_l.*gerr_rif.data; 

        % Error integral is stored for final performance evaluation

        for i = 1:N
            err_ev(i,iteration) = trapz(err_rif.time, abs(err_rif.data(:,i)))/t_fin;
        end

        % Control integral is stored for final performance evaluation

        qb_input_app = timeseries(qb_input.signals.values((i_0:i_f), :), time);
        qb_input_app = resample(qb_input_app, time(1): period : time(end));


        qb_in = timeseries(qb_input_app.data(ind_0 : ind_f, :), qb_input_app.time(ind_0 : ind_f));
        qb_in = resample(qb_in, qb_in.time(1): T_sample : qb_in.time(end));


        for i = 1:N
            cont_ev(i,iteration) = trapz(qb_in.time, abs(qb_in.data(:,i)))/t_fin;
        end

        % Control inputs are assigned to simulink variables

        FF.signals.values = zeros(length(FF.time),N);
        for i = 1:N
            FF.signals.values(1 : (pre_time/T_sample), i) = u_err_resampled.data(1:(pre_time/T_sample) , i);
            FF.signals.values((pre_time/T_sample + 1) : (pre_time + t_fin)/T_sample  + 1, i) = control(:, i); 
        end

        for i = ((pre_time + t_fin)/T_sample + 2):length(FF.signals.values)
            FF.signals.values(i, :) = FF.signals.values((pre_time + t_fin)/T_sample  + 1, :);
        end     

        %% Storing data
        cd(folder)

        name = 'ws_';
        name = strcat(name,num2str(tot_iter + iteration));
        save(name);
        cd ..
        cd ..

        if (mod(iteration, 5) == 0) && (iteration ~= max_iter)
            save_data(max_iter, (tot_iter + iteration), 0);
            if(check_error_threshold(N, err_ev(:, iteration), error_threshold))
                disp('Error threshold achieved');
                break;
            end
        end
    end
    
    % Update the total number of iterations
    tot_iter = tot_iter + iteration;
    
    %% Output

    FF_out = FF; 

    if err_tot == 0
        err_tot = zeros(N, tot_iter);
        cont_tot = zeros(N, tot_iter);
        err_tot = err_ev(:,1:iteration);
        cont_tot = cont_ev(:,1:iteration);
    else
       err_app =  err_tot;
       cont_app = cont_tot;
       err_tot = zeros(N, tot_iter);
       cont_tot = zeros(N, tot_iter);
       err_tot = horzcat(err_app, err_ev(:,1:iteration));
       cont_tot = horzcat(cont_app, cont_ev(:,1:iteration));
    end

    cd(folder)
    name = 'ws_';
    name = strcat(name,num2str(tot_iter));
    save(name);
    cd ..
    cd ..
    
    % Plot final results
    ILC_print = 1;
    if ILC_print == 1
        save_data(iteration, tot_iter, 1);
        disp(strcat(10, 'Executed a set of ', 9, num2str(iteration), ' iterations'));
    end
end