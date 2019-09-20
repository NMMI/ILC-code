%% Save figures to track learning performances

function y = save_data(max_iter, iter, flag)
    global N t_fin pre_time post_time T_sample samples Amp_step step_period...
           Kpon Kdon Kpoff Kdoff  x_0 dx_0 preset FF u_err_resampled TT_FF0 step_time... 
           u_int ref dref ddref r dr ddr preset_inertia s_t control...
           tot_iter folder err_tot cont_tot err_ev cont_ev ind_0 ind_f i_0 i_f...
           err_rif err_rif_app gerr gerr_rif time  err derr qb_input u_plot q_plot r_plot   

% Uncomment the following line if matlab does not find function 'create_legend_name.m'
%     copyfile('./Functions/create_legend_name.m',folder);
       
	cd(folder)

	if flag == 1
        figure
        plot(1:tot_iter, err_tot);
        title('Errors evolution');
        legend(create_legend_name('err', N));
        xlabel('iteration');
        ylabel('q_{err} [rad]');

        name = 'err_tot_';
        name = strcat(name,num2str(tot_iter));
        saveas(gcf, name);



        figure
        plot(1:max_iter, err_ev(:,1:max_iter));
        title('Errors evolution');
        legend(create_legend_name('err', N));
        xlabel('iteration');
        ylabel('q_{err} [rad]');

        name = 'err_';
        name = strcat(name,num2str(tot_iter));
        saveas(gcf, name);



        figure
        plot(1:max_iter, cont_ev(:,1:max_iter));
        title('Controls evolution');
        legend(create_legend_name('qb_{input}', N));
        xlabel('iteration');
        ylabel('qb_{input} [rad]');

        name = 'cont_';
        name = strcat(name,num2str(tot_iter));
        saveas(gcf, name);


        figure
        plot(1:max_iter, sum(err_ev(:,1:max_iter),1)/N);
        title('Error evolution');
        legend('err');
        xlabel('iteration');
        ylabel('q_{err}[rad]');

        name = 'err_int_';
        name = strcat(name,num2str(tot_iter));
        saveas(gcf, name);


        figure
        plot(1:max_iter, sum(cont_ev(:,1:max_iter),1));
        title('Control evolution');
        legend('qb_{input}');
        xlabel('iteration');
        ylabel('qb_{input}[rad]');

        name = 'cont_int_';
        name = strcat(name,num2str(tot_iter));
        saveas(gcf, name);

    end
       
    
    figure
    plot(q_plot.time, q_plot.signals.values, r_plot.time, r_plot.signals.values);
    title('Trajectory Tracking');
    legend([create_legend_name('q', N); create_legend_name('r', N)]);
    xlabel('time [s]');
    ylabel('evolution [rad]');
    
    
    name = 'trajectory_';
    name = strcat(name,num2str(iter));
    saveas(gcf, name);
    if flag == 0
        close(gcf);
    end
    
    figure
    plot(q_plot.time, (r_plot.signals.values - q_plot.signals.values ));
    title('Errors evolution over time');
    legend(create_legend_name('e', N));
    xlabel('time [s]');
    ylabel('evolution [rad]');
    
    name = 'err_time_';
    name = strcat(name,num2str(iter));
    saveas(gcf, name);
    
    if flag == 0
        close(gcf);
    end
    
    figure
    plot(u_plot.time, u_plot.signals.values);
    title('Feedback');
    legend(create_legend_name('u', N));
    xlabel('time [s]');
    ylabel('u [rad]');
    
    name = 'u_';
    name = strcat(name,num2str(iter));
    saveas(gcf, name);
    
    if flag == 0
        close(gcf);
    end
    
    figure
    plot(FF.time, FF.signals.values)
    title('Feedforward');
    legend(create_legend_name('FF', N));
    xlabel('time [s]');
    ylabel('FF [rad]');
    
    name = 'FF_';
    name = strcat(name,num2str(iter));
    saveas(gcf, name);

    if flag == 0
        close(gcf);
    end
    
	cd ..
    cd ..

    y = max_iter;

end

