function y = check_error_threshold(N, errors, threshold)
    y = 1;
    for i=1:N
        if(errors(i) > threshold)
            y = 0;
        end
    end
end

