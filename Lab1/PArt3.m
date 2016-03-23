%Part 3
%Testing
%Benchmarking

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
%iterQ_Control is a vector each elelemt corresponds to the number of
%iterations for h
[h,Q,time,iterQ_Control]=ControlLawIterationNoVisualize(gamma,gridSize,nrOfActions,error,model);

iterQ_Control

tic
ControlLawIterationNoVisualizeForTime(gamma,gridSize,nrOfActions,error,model);
toc

[hmat,Q,iterQ_Q] = QIterationNoVisualize( gamma,gridSize,nrOfActions, error, model);

iterQ_Q

tic
QIterationNoVisualizeForTime( gamma,gridSize,nrOfActions, error, model);
toc
