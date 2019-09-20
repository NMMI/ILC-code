function y = f_eq(x, I, b, r, dr, ddr, D)
    cd Parameters
    cube_parameters;
    cd ..
    a = (a1 + a2)/2;
    k = (k1 + k2)/2;
    y = I*ddr + b*dr - (2*k*cosh(a*D)*sinh(a*(x - r)) + m*(x - r));
end
