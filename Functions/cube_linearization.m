%% Linearize along a trajectory the qbmove dynamics 

function [A,B] = cube_linearization(u)
    I = u(1);
    b = u(2);
    q_eq = u(3);
    dq_eq = u(4);
    sum_eq = u(5);
    dif = u(6);
    delta = u(7:8);
    
    % Define the symbolic variables
    q = sym('q','real');
    dq = sym('dq','real');
    sum = sym('sum','real');
    
    % Define the order of variables
    z = [q, dq]';
    
    cd Parameters
    cube_parameters;
    cd ..
    
    k = (k1 + k2)/2;
    a = (a1 + a2)/2;
    
    tau = 2*k*cosh(a*dif)*sinh(a*(sum - q)) + m*(sum - q);
    f = [0, 1; 0, -b/I]*z + [0; 1/I]*tau + delta;

    % Linearization

    % Jacobians computation
    A1 = jacobian(f, z);
    B1 = jacobian(f, sum);
    
    % Evaluation at equilibrium point
    A2 = subs(A1, sum, sum_eq);
    B2 = subs(B1, sum, sum_eq);
    A3 = subs(A2, {q, dq}, {q_eq, dq_eq});
    B3 = subs(B2, {q, dq}, {q_eq, dq_eq});
    A = double(A3);
    B = double(B3);
end
