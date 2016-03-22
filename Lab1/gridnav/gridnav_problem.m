function out = gridnav_problem(mode, varargin)
% RL problem setup function for the grid navigation problem.
%   OUT = GRIDNAV_PROBLEM(MODE, [PARAM1, [PARAM2, ...]])
%
% Parameters:
%   MODE        - specifies what data should be returned. Possible values: 'model', 'info', 'reset'.
%   PARAM1, ... - (optional) extra parameters. Can be used to further configure the creation of
%       the data.
% Returns:
%   OUT         - depending on the value of mode, this should be as follows:
%       - MODE = 'info', information about the model. Representative fields:
%           id      - problem identifier
%           problem - handle to model function
%           det 	- whether problem is deterministic or stochastic
%           p       - # of state variables
%           q       -  # of control variables
%
%       - MODE = 'model', the model structure. Representative fields:
%           p       - number of states
%           q       - numer of inputs (controls)
%           nw       - number of noise variables (if any)
%           fun     - the mdp function (dynamics and rewards) of the process
%           Ts      - discretization step
%       The following can be configured:
%           size    - the grid size, a vector [width, height]
%           x_goal  - the location of the goal, a column vector containing the 2 coordinates
%           x_obst  - the locations of obstacles, one per column, each in the same format as the goal
%           rew_step, rew_obst, rew_goal
%                   - the rewards obtained on, respectively, making a regular step, hitting an
%                   obstacle, and getting to the goal
%
%       - MODE = 'reset', returns a reset state in the beginning of a trial. Called as follows:
%       GRIDNAV_PROBLEM('reset', model, reset) where reset can be 'rand' for random reset to a
%       nonterminal state, or a matrix containing reset states in the columns; the choice between
%       these reset states will be random in this case
% 
%  The state is the 2-D position on the grid, and the action ranges in {1, 2, 3, 4}, signifying
%  respectively, {move left, right, down, up}.


% END sample_problem() RETURNING out ===================================================
switch mode,

    % Offer basic info about the model (without creating the actual model)
    case 'info',    
        info.id = 'gridnav';               % an identifier
        info.problem = @gridnav_problem;   % handle to model function
        info.det = 1;                      % can be either deterministic or stochastic
        info.p = 2;                        % # of state variables
        info.q = 1;                        % # of control variables
        out = info;
        
    % create the model
    case 'model',
        model.p = 2;                        % number of states
        model.q = 1;                        % number of control inputs (actions)
        model.det = 1;
        % model.nw = 0;                     % number of noise variables
        model.Ts = 1;                       % no sample time per se, just a discrete time index
        model.fun = 'gridnav_mdp';          % MDP function
        % visualization function
        model.visualizefun = 'cleanrob_visualize';
        model.U = {1:4};
        model.Udeltax = [-1 0; 1 0; 0 -1; 0 1]';
        
        % configurable properties
        CFG.size = [5 5];                 
        CFG.x_goal = [5; 5];
        CFG.x_obst = [];
        CFG.rew_step = -.1;
        CFG.rew_obst = -.1;
        CFG.rew_goal = 10;
        
        if nargin >= 2, cfg = varargin{1};
        else cfg = struct; 
        end;
        cfg = parseconfig(cfg, CFG);
        model = copyfields(cfg, model);
        
        model.X = {1:cfg.size(1), 1:cfg.size(2)};
        if isempty(model.x_obst), model.x_obst = [-1; -1]; end; % easier to handle later, with findflat
        
        % determine terminal states, marking them in a flat vector
        model.Xflat = flat(model.X);
        model.Xterminal = zeros(1, size(model.Xflat, 2));
        % goal is terminal
        model.Xterminal(findflat(model.x_goal, model.Xflat)) = 1;
        % obstacles are terminal (for safety reasons; they should never be reached in practice)
        for i = 1:size(model.x_obst, 2),
            model.Xterminal(findflat(model.x_obst(:, i), model.Xflat)) = 1;
        end;

        % determine minima and maxima
        model.minx = [1; 1]; model.maxx = model.size(:);
        model.minu = min(model.U{1}); model.maxu = max(model.U{1});
        
        out = model;
        
    % reset to initial state
    case 'reset',
        model = varargin{1};
        reset = varargin{2};
        if isempty(reset), reset = 'rand'; end;
        if isnumeric(reset),
            if size(reset, 2) > 1,      x0 = reset(:, unidrnd(size(reset, 2)));
            else                        x0 = reset;
            end;
        elseif strcmp(reset, 'rand'),
            % reset to random NONTERMINAL state
            X0 = model.Xflat(:, ~model.Xterminal);
            x0 = X0(:, unidrnd(length(X0)));
        end;
        
        out = x0;

end;        % mode SWITCH

% END sample_problem() RETURNING out ===================================================