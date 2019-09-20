function y = calc_Delta(u)
    I = u(1);
    b = u(2);
    q_0 = u(3);
    dq_0 = u(4);
    sum_0 = u(5);
    dif = u(6);
    
    cd Parameters
    cube_parameters;
    cd ..
    
    k = (k1 + k2)/2;
    a = (a1 + a2)/2;
    
    tau = 2*k*cosh(a*dif)*sinh(a*(sum_0 - q_0)) + m*(sum_0 - q_0);
    y = [-dq_0; (b*dq_0 - tau)/I];
end

