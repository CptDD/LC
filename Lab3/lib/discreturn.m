function R = discreturn(cfg, r, Ns, terminal, appendinfhor)
% Computes discounted return of states along a trajectory from the instantaneous rewards
% Parameters:
%   CFG         - the config, should include gamma
%   R           - the reward vector
%   Ns          - the number of time samples at which control is applied
%   TERMINAL    - signals whether the task has reached a terminal state at
%       the end of the reward vector
%   APPENDINFHOR- if TERMINAL=0 and this flag is set to 1, an
%       infinite-horizon reward equal to the last reward divided by
%       (1-gamma) will be appended at the end of the reward vector. This
%       assumes the reward will remain constant until infinity. The default
%       value of this flag is 0 (do not append infinite horizon reward)
%
% Returns:
%   R           - the return vector for each state in the trajectory

if nargin < 5,
    appendinfhor = 0;
end;
appendinfhor = ~terminal && appendinfhor;

% compute returns vector assuming the reward remains constant up to infinity
% discounting vector starting from t=0
discounting = cfg.gamma .^ (0:Ns); % 0:Ns-1 for the rewards; Ns for the infinity reward
% discounted rewards for t=0; append also the infinity return: r_inf*1/(1-gamma) for
% continuing tasks, 0 for episodic tasks that ended
% Note episodic tasks are identified by whether the last state is terminal; if the task did
% not terminate in the replay but is episodic, the returns will be computed incorrectly
R = [r(2:Ns+1) (appendinfhor * r(end))/(1-cfg.gamma)] .* discounting;
% [gamma^0*R_0 gamma^1*R_1 ... gamma^K*R_K gamma^K+1*Rinf] (need to reverse to do the cumulative sum)
R = cumsum(R(end:-1:1));
% divide R_k by gamma^k for each k,to find out return starting from k
R = R(end:-1:1) ./ discounting;
