%% compute a near-optimal solution for the gridnav problem

% configure the model here
mcfg.x_obst= [1 3 4 ; 3 2 5];
mcfg.rew_step=-.1; 
mcfg.rew_obst=-.1;
mcfg.x_goal=[5;5];
mcfg.rew_goal=10;

% configure policy iteration
qicfg.run = 1;
qicfg.gamma = 0.98;
qicfg.eps = 1e-6;
qicfg.model_params = {mcfg};
qicfg.problem = 'gridnav_problem';
qicfg.verb = 0;
Qstar = reshape(qiter(qicfg), 5, 5, 4);