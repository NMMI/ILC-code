%% model_parameters

global N

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
N = 6; % Number of joints
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

out_int = zeros(N,1);           % Integrators initial condition
Activator = ones(N,1);          % qbmoves activation input
Integral_action = 2*ones(N,1);	% Used to compute the positioning control action