%% Create a new folder to store the data

function path = create_new_folder()
    cd Images % all the data folders are in ./Images
    current_time = clock; % get the date and the time
    dir = num2str(current_time(1));
    for i = 2:5
      dir = strcat(dir,'_',num2str(current_time(i))); 
    end
    mkdir(dir); % Create a directory named 'yyyy_mm_dd_hh_mm'
    cd ..
    path = strcat('Images/',dir);
end