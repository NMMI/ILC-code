%% step parameters

global N

Amp_step = -0.3*ones(N,1);          % rad
step_period = 0.004;                % s
selector = zeros(N,1);

ARX_model = 0; % Bool variable used to select between two methods: 1 = ARX model and Forward Euler, 0 = direct evaluation through damping factor and natural frequency