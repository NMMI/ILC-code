%% Evaluate ILC offline parameters

function [p, d] = calc_Koff(B)
    global N
    K = pinv(B); % Moore-Penrose pseudoinverse
    p = zeros(N,1);
    d = zeros(N,1);
    for i = 1:N
        p(i) = K(i,2*i-1); 
        d(i) = K(i, 2*i);
    end
end

