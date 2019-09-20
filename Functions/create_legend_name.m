%% Create figure legend

function legend_name= create_legend_name(name, N)
 legend_name = cell(N,1);
    for i = 1:N
        legend_name(i) = {strcat(name, '(', num2str(i), ')')};
    end
end