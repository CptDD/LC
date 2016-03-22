% create a model of the grid nav problem, see also help gridnav_problem
% the size along the X and Y coords respectively
global viscfg
gridSize=[5 5];
nrOfActions=4;
cfg.size = gridSize;
% an obstacle at coordinates (X=3,Y=3); note it is given as a column vector
cfg.x_obst = [1 3 4 4; 3 2 5 4];
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
    
     % show the updated state of the agent
viscfg = struct;
viscfg.model = model;


x_start=[3;1];
nrOfActions=4;
maxIter=20;

NaiveFunction(x_start,maxIter,nrOfActions,model);

