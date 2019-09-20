%% Riccati equation
function Srdot = myRiccati(t, Sr, A, B, Q, rho, time)
    array_1 = squeeze(A(2,1,:));
    array_2 = squeeze(B(2, 1, :));

    a = interp1(time, array_1, t);
    b = interp1(time, array_2, t);

    F = zeros(2);
    G = zeros(2,1);

    G = [B(1,1,1); b];
    F = [A(1, 1, 1), A(1, 2, 1); a, A(2, 2, 1)];

    Sr = reshape(Sr, size(F)); %Convert from "n^2"-by-1 to "n"-by-"n"
    Srdot = -F.'*Sr - Sr*F + Sr*G*rho^-1*G.'*Sr - Q; %Determine derivative
    Srdot = Srdot(:); %Convert from "n"-by-"n" to "n^2"-by-1
end