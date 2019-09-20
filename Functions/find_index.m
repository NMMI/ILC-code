function index = find_index(time, value, tol, direction)
    len = length(time);
    if direction == 1
        for i = 1:len
            if abs(time(i) - value) < tol
               index = i;
               return;
            end
        end
    elseif direction == -1
        for i = len : -1 : 1
            if abs(time(i) - value) < tol
                index = i;
                return;
            end
        end
    end
    index = NaN;
end

