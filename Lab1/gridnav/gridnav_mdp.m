function [xplus, rplus, terminal, pplus] = gridnav_mdp(m, x, u)
%  [XPLUS, RPLUS, TERMINAL] = GRIDNAV_MDP(M, X, U)
%  Parameters:
%   M   - the model specification
%   X   - current state, x(k)
%   U 	- command u(k)
%  Returns:
%   XPLUS       - state at next sample, x(k+1)
%   RPLUS       - ensuing reward, r(k+1)
%   TERMINAL    - whether the state is terminal (i.e., end of episode)
%               Should always be 0 for a continuing task
%   PPLUS       - probability of next state

% special case: terminal state -- cannot be changed and always leads to 0 reward
if all(x == m.x_goal) || ~isempty(findflat(x, m.x_obst)), 
    xplus = x;
    rplus = 0;
    terminal = 1;
    if nargout == 4, pplus = 1; end;
    return;
end;

udeltax = m.Udeltax(:, u);
xplus = sat(x+udeltax, m.minx, m.maxx);

if all(xplus == m.x_goal),
    % reached goal
    rplus = m.rew_goal;
    terminal = 1;
elseif ~isempty(findflat(xplus, m.x_obst)),
    % hit an obstacle, reset to earlier position
    xplus = x;
    rplus = m.rew_obst; 
    terminal = 0;
else
    % regular move
    rplus = m.rew_step;
    terminal = 0;
end;    

if nargout == 4, pplus = 1; end;


end % main function
