%% Store control parameters

global samples Kpon Kdon Kpoff Kdoff t_fin pre_time FF folder 

% Uncomment this line if matlab does not find functions create_legend_name.m
% copyfile('./Functions/create_legend_name.m',folder);

cd(folder)

save('ws_init');

figure
plot(FF.time((pre_time/T_sample + 1) : (pre_time + t_fin)/T_sample + 1), Kpoff)
title('Kpoff')
legend(create_legend_name('Kpoff', N));
xlabel('time [s]');
ylabel('value');
        
name = 'Kpoff';
saveas(gcf, name);

figure
plot(FF.time((pre_time/T_sample + 1) : (pre_time + t_fin)/T_sample + 1), Kdoff)
title('Kdoff')
legend(create_legend_name('Kdoff', N));
xlabel('time [s]');
ylabel('value');
        
name = 'Kdoff';
saveas(gcf, name);

figure
plot(FF.time, Kpon.signals.values)
title('Kpon')
legend(create_legend_name('Kpon', N));
xlabel('time [s]');
ylabel('value');
        
name = 'Kpon';
saveas(gcf, name);

figure
plot(FF.time, Kdon.signals.values)
title('Kdon')
legend(create_legend_name('Kdon', N));
xlabel('time [s]');
ylabel('value');
        
name = 'Kdon';
saveas(gcf, name);

figure
plot(preset.time, preset.data)
title('preset')
legend(create_legend_name('d', N));
xlabel('time [s]');
ylabel('value [rad]');
        
name = 'preset';
saveas(gcf, name);

figure;
plot(FF.time, FF.signals.values)
title('Feedforward');
legend(create_legend_name('FF', N));
xlabel('time [s]');
ylabel('FF [rad]');

name = 'FF_1';
saveas(gcf, name);
        
save('reference', 'r', 'dr', 'ddr')

cd ..
cd ..
