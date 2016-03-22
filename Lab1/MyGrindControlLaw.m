% create a model of the grid nav problem, see also help gridnav_problem
% the size along the X and Y coords respectively
global viscfg
gridSize=[8 9];
nrOfActions=4;
cfg.size = gridSize;
% an obstacle at coordinates (X=3,Y=3); note it is given as a column vector
cfg.x_obst = [1 3 4 4 7 6 8 ; 3 2 5 4 8 3 9];
% the goal location is at (X=5,Y=5)
cfg.x_goal = [5; 5];
% we could also configure the rewards: per regular step, field rew_step
% for collision with obstacle or wall: rew_obst; and for reaching the goal: rew_goal
model = gridnav_problem('model', cfg);

% use the created to simulate a transition
% start from X=3, Y=1 and go right (u=2)
% xplus is the next state, rplus the reward resulting from the transition
% terminal is a boolean flag indicating whether the resulting state is
% terminal or not

%     [xplus, rplus, terminal] = gridnav_mdp(model, [2; 1], 2);   
%     xplus
%     rplus
%     
%      % show the updated state of the agent
    viscfg = struct;
    viscfg.model = model;
%     viscfg.x = xplus;
%     viscfg.gview = gridnav_visualize(viscfg);    % note we remember the view...
    
gamma=0.7;
error=0.1;
[h,Q]=ControlLawIteration(gamma,gridSize,nrOfActions,error,model);


% to show a Q-function, we place it on viscfg
viscfg.x = [];  % before, remove the robot state since we don't want to show it anymore
% as an example, we initialize an arbitrary Q-function
% (in your solution the Q-function will be computed by the algorithm)
% note that Q must be an array with standard dimensions in order to be handled
% correctly by the visualization function:
% size on X (here, 5) x size on Y (here, 5) x number of actions (here, 4)
% n (= number of states = 5) rows, and 2 (number of actions) columns
viscfg.Q = Q;
viscfg.gview = gridnav_visualize(viscfg);   

% to show a policy in addition to the Q-function, add it to viscfg 
% h also has a standard structure: a matrix with 
% (size on X x size on Y) elements, each representing an action, with values 1 to 4


viscfg.h = h;
% if we wanted to NOT reuse the view, but create a new figure, we could do:
%viscfg.gview = [];
viscfg.gview = gridnav_visualize(viscfg);
