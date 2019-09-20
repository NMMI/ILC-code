function [I, bet, kap] = calc_inertia_and_damping(u)
    global N samples t_fin pre_time T_sample step_time u_err_resampled step_res u_step...
          Amp_step step_period u_int s_t preset preset_inertia selector

    cd Parameters
    step_parameters;
    cd ..
    
    u_int.time = [0:T_sample:(step_time(1) + pre_time)]';
    u_int.signals.values = zeros(length(u_int.time),N);   
    u_int.signals.dimensions =  N;
    
    preset_inertia = preset;
        
    for i = 1:N
        u_int.signals.values((pre_time/T_sample + 1) : end, i) = u(i);
        u_int.signals.values(1 : (pre_time/T_sample), i) = u_err_resampled.data(1: length(u_int.signals.values(1 : (pre_time/T_sample), i)), i);
        preset_inertia.data((pre_time/T_sample + 1) : end, i) = preset_inertia.data(pre_time/T_sample, i);
    end
    
    period = 0.0001;
    tolerance = period - period*0.1;
    x = zeros(N,1);
    y = zeros(N,1);
    z = zeros(N,1);
    I = zeros(N,1);
    bet = zeros(N,1);
    kap = zeros(N,1);
 
    s_t = 0;
    
    for i = 1:N
        s_t = step_time(i);
        
        selector = zeros(N,1);
        selector(i) = Amp_step(i);
    
        % Step input 
        sim('Simulink/Evaluate_inertia_and_damping');
        
        pause on;
        pause(5);
        pause off;

        u_step.signals.values = circshift(u_step.signals.values(:, i), 1); % Adding one step delay to system input


        ind = find(step_res.time >= pre_time, 1);
        ind = ind - 1;

        step_t = step_res.time(ind : end);

        step_res.signals.values(ind:end,i) = step_res.signals.values(ind:end,i) - step_res.signals.values(ind,i); 


        step_rif_app = timeseries(step_res.signals.values((ind:end), i), step_t);
        step_rif_app = resample(step_rif_app, step_t(1): period : step_t(end));

        step_ind = find_index(step_rif_app.time, pre_time, tolerance, 1);

        output = timeseries(step_rif_app.data(step_ind : end), step_rif_app.time(step_ind : end)  - pre_time);
        output = resample(output, output.time(1): step_period: output.time(end));
        
        if ARX_model == 1         % Evaluate parameters through ARX model and Forward Euler

            ind = find(u_step.time >= pre_time, 1);
            ind = ind - 1;

            step_t = u_step.time(ind :end);

            step_rif_app = timeseries(u_step.signals.values((ind:end)), step_t);
            step_rif_app = resample(step_rif_app, step_t(1): period : step_t(end));

            step_ind = find_index(step_rif_app.time, pre_time, tolerance, 1);

            input = timeseries(step_rif_app.data(step_ind : end), step_rif_app.time(step_ind : end)  - pre_time);
            input = resample(input, input.time(1): step_period: input.time(end)); 
                
            data = iddata(output.data(:,:),input.data(:,:),step_period);
            m = arx(data,[2 1 1]);

            x(i) = m.B(2);
            y(i) = m.A(2);
            z(i) = m.A(3);

            I(i) = (step_period^2)/x(i);                              % inertia
            bet(i) = (y(i) + 2)*I(i)/step_period;                     % damping
            kap(i) = ((z(i) - 1)*I(i) + bet(i)*step_period)/(step_period^2); 
            
        else	% Evaluate parameters through damping factor and natural frequency

            % 2% settlingtime 
            S_info(i) = stepinfo(output.data, output.time, output.data(end),'SettlingTimeThreshold',0.02);            
            over = S_info(i).Overshoot/100;
            kap(i) = Amp_step(i)/output.data(end);
            smorzamento(i) = sqrt((log(over)^2)/(pi^2 + log(over)^2));
            pulsazione(i) = 3.9/(smorzamento(i)*S_info(i).SettlingTime);
            bet(i) = (2*smorzamento(i)*kap(i))/pulsazione(i);   % damping
            I(i) = kap(i)/(pulsazione(i)^2);                    % inertia
        end
        
    end
end

