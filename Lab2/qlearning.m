function [Qseq, Rseq] = qlearning(config)
% Template for Q-learning function
% Inputs: assumed here to be given as a configuration structure "config", which 
% should contain the following fields:
%   gamma = discount factor
%   alpha = learning rate
%   epsilon = exploration probability
%   epsilondecay = exploration probability decay rate (when this feature is implemented)
%   T = number of trials
%   K = maximum number of steps in every trial
%   visualize = a boolean value telling the function whether visualization should be enabled
% Outputs: Qseq, Rseq
%   Qseq could be an array with dimension (T, dimx1, dimx2, dimu) where dimxi are the numbers of
%   discrete states of the problem along each dimension i and dimu the number of discrete actions
%   Rseq could be a vector of dimension T containing the return for each trial

% initialization:
global viscfg
model=viscfg.model;
% create a model, startup the visualization if enabled, etc.
Q=zeros(config.gridSize(1),config.gridSize(2),config.nrOfActions);
Qseq=cell(config.T);
Rseq=zeros(1,config.T);
terminal=0;
for i=1:config.T
    xk=[1;1];
    k=0;
    Rtemp=0;
    terminal=0;
    while(k<=config.K)&&(terminal~=1)
        %get a random value to use epsilon
        prob=rand();
        if (prob<config.epsilon)% when this value is less than epsilon
            uk=randi(4);%get a random control action
        else
            [aux,uk]=max(Q(xk(1),xk(2),:));%get the control value from Q
            
        end
        [xplus, rplus, terminal] = gridnav_mdp(model, xk, uk);
        %calculating the new Q   
        Q(xk(1),xk(2),uk)=Q(xk(1),xk(2),uk) + config.alpha*(rplus + config.gamma*(max(Q(xplus(1),xplus(2),:))) - Q(xk(1),xk(2),uk) );
        %this part is for visualization
        if (config.visualize==1)
            viscfg.x=xplus;
            %viscfg.Q = Q;
            viscfg.gview = gridnav_visualize(viscfg);  
        end
        k=k+1;
        xk=xplus;
        Rtemp=Rtemp + config.gamma^k*rplus;
    end
    viscfg.Q = Q;
    viscfg.gview = gridnav_visualize(viscfg); 
    Rseq(i)=Rtemp;
    Qseq{i}=Q;
end
% (see the examples and your solutions to earlier labs for ideas)

% run T trials, choosing actions with epsilon-greedy and updating the Q function at each step 
% after each step, update the visualization if enabled
% Hints:
%   - reuse the code you implemented to simulate a trajectory in lab 1
%   - to implement an instruction "with probability epsilon", use:
%       if rand <= epsilon, instruction (since rand gives a uniformly distributed number in [0, 1])

% at the end of the trial:
% save the updated Q function on the sequence
% save the return on the sequence
% decay the exploration probability
% if visualization is enabled, show the new Q-function and the corresponding greedy policy on the visualization
end