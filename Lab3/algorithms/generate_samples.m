function samples = generate_samples(model, cfg, approx, onlyparseconfig)
% Generate state-action samples for sample-based batch DP/RL algorithms.
% Used in the estimation of Q-functions.
% If onlyparseconfig is set, only parses and returns the configuration.

% Default parameters
CFG.N = [];            % # of samples to generate
CFG.method = 'randdisc';
    % Supported methods:
    %   'rand[disc]'    - uniformly distr. continuous states and continuous [or discrete] actions
    %   'randndisc'     - normally distr. continuous states and discrete actions
    %   'randh[disc]'   - uniformly distr. continuous states and [discretized] actions given by policy
    %   'traj[disc]'    - controlled trajectory [with discretized actions]
    %   'grid'          - simple grid of state-action pairs
CFG.xgrids = [];        % x grids -- or counts -- for grid sampling
CFG.ugrids = [];        % for discrete-action or grid sampling
CFG.gridfun = @symequidgrid;
CFG.policy = [];        % policy to use in generating samples
CFG.policyargs = {};    % extra arguments to policy
CFG.explor = 0.5;       % exploration probability when using policy
% *traj* options
CFG.traj_X0 = [];       % 'rand' or set of values
CFG.traj_maxx0 = [];    % rand initial state in this domain rather than the whole X
CFG.traj_K = Inf;            % number of steps in each trajectory (default single trajectory)
CFG.sigmax = [];
% Currently unused fields:

% also support single, flat-argument call with # of samples
if isnumeric(cfg) && numel(cfg) == 1,
    N = cfg; cfg = struct; cfg.N = N;
end;
cfg = parseconfig(cfg, CFG);

% return the parsed config if set to do so
if nargin > 3 && onlyparseconfig, samples = cfg; return; end;

% shorthand vars
p = model.p; q = model.q; fun = model.fun;
maxx = model.maxx(:); maxu = model.maxu(:);
N = cfg.N;
% prepare auxiliary vars
% obtain discrete U when necessary
switch cfg.method,
    case {'randdisc', 'randndisc', 'randhdisc', 'trajdisc'}, % , 'expdisc'}
        if isempty(cfg.ugrids),
            % need to obtain grids from the approximator
            if ~isfield(approx, 'U') || isempty(approx.U),
                error('Could not obtain U grids');
            end;
            cfg.ugrids = approx.U;
        end;
        disc = 1;               % flag using discrete method
        ugrids = cfg.ugrids;    % shorthand var
        % compute size of grids
        un = zeros(length(ugrids)); for i=1:length(ugrids), un(i) = length(ugrids{i}); end;
        Uflat = flat(ugrids); 
        M = size(Uflat, 2);
    otherwise,
        disc = 0;               % flag using continuous method
end;        

% init samples arrays
if ~strcmp(cfg.method, 'grid'),    % "grids" method initializes the arrays separately
    X = zeros(p, N); U = zeros(q, N); Xp = X; R = zeros(1, N); T = zeros(1, N);
end;

% generate samples according to method
switch cfg.method,
    case 'rand',
        for i = 1:N,
            X(:, i) = (2*rand(p, 1) - 1) .* maxx;
            U(:, i) = (2*rand(q, 1) - 1) .* maxu;
            [Xp(:, i), R(i), T(i)] = fun(model, X(:, i), U(:, i));
        end;

    case 'randdisc',
        for i = 1:N,
            X(:, i) = (2*rand(p, 1) - 1) .* maxx;
            U(:, i) = Uflat(:, unidrnd(M));
            [Xp(:, i), R(i), T(i)] = fun(model, X(:, i), U(:, i));
        end;

    case 'randndisc',
        sigmax = cfg.sigmax(:);
        i = 1;
        while i <= N,
            X(:, i) = randn(p, 1) .* sigmax;
            if any(abs(X(:, i)) > maxx), continue; end; % reject sample
            U(:, i) = Uflat(:, unidrnd(M));
            [Xp(:, i), R(i), T(i)] = fun(model, X(:, i), U(:, i));
            i = i + 1;
        end;
        
    case {'randh', 'randhdisc'},
        % determine the order of the possibly dynamic controller
        try     [cx0, cord] = feval(cfg.policy, 'init', cfg.policyargs{:});
        catch   cord = 0;   % compatibility for old-style static policies
        end;
        if cord > 0, error('Cannot control independent samples with a dynamical controller'); end;
        for i = 1:N,
            X(:, i) = (2*rand(p, 1) - 1) .* maxx;
            if rand >= cfg.explor,  % exploit: use action from policy
                U(:, i) =  feval(cfg.policy, X(:, i), [], cfg.policyargs{:});    % [] = undefined time argument
                if disc, U(:, i) = quantize(U(:, i), ugrids, un); end;
            else                    % explore: use random action
                if disc,    U(:, i) = Uflat(:, unidrnd(M));
                else        U(:, i) = (2*rand(q, 1) - 1) .* maxu;
                end;
            end;
            [Xp(:, i), R(i), T(i)] = fun(model, X(:, i), U(:, i));
        end;
        
    case {'traj', 'trajdisc'},
        % determine the order of the possibly dynamic controller
        try     [cx0, cord] = feval(cfg.policy, 'init', cfg.policyargs{:});
        catch   cord = 0;   % compatibility for old-style static policies
        end;
        if cord > 0, error('Dynamical controller not yet implemented'); end;
        % process initial state (trajectory reset)
        if ischar(cfg.traj_X0) && strcmp(cfg.traj_X0, 'rand'),
            userandx0 = 1;
            % domain for random initial state: either specified on config, or whole state space
            if ~isempty(cfg.traj_maxx0),maxx0 = cfg.traj_maxx0(:);
            else                        maxx0 = maxx;
            end;
        else
            userandx0 = 0;
            % if character identifier, try getting the actual states using the problem fun
            if ischar(cfg.traj_X0), X0 = feval(model.problem, 'X0', cfg.traj_X0); 
            else                    X0 = cfg.traj_X0;       % assume actual states
            end;
            % if cell array, flatten
            if iscell(X0), X0 = flat(X0); end;
            N0 = size(X0, 2);
            i0 = 1;         % index of current initial state (trajectory)
        end;
        K = cfg.traj_K;      % max length of trajectory
        i = 1;          % current sample index
        while i <= N,
            % initialize state for current trajectory
            if userandx0,
                X(:, i) = (2*rand(p, 1) - 1) .* maxx0;
            else
                X(:, i) = X0(:, i0);
                % iterate through initial states, rotating back to 1st at the end
                i0 = mod(i0+1 - 1, N0) + 1;
            end;
            % control the system for at most K steps
            for k = 1:K,
                if rand > cfg.explor,  % exploit: use action from policy
                    U(:, i) =  feval(cfg.policy, X(:, i), k*model.Ts, cfg.policyargs{:});
                    if disc, U(:, i) = quantize(U(:, i), ugrids, un); end;
                else                    % explore: use random action
                    if disc,    U(:, i) = Uflat(:, unidrnd(M));
                    else        U(:, i) = (2*rand(q, 1) - 1) .* maxu;
                    end;
                end;
                [Xp(:, i), R(i), terminal] = fun(model, X(:, i), U(:, i));
                T(i) = terminal;
                i = i + 1;
                if terminal || i > N, 
                    break;  % trajectory completed (or samples completed)
                else        % current x+ becomes x for the next sample
                    X(:, i) = Xp(:, i - 1);
                end;
            end;
        end;
       
    case 'grid',        % grid (cross-product of individual arrays per dimension) of samples
        if length(cfg.xgrids) ~= p, error('There should be "p" X grids or grid lengths'); end;
        if length(cfg.ugrids) ~= q, error('There should be "q" U grids or grid lengths'); end;
        % check if grid counts are given instead of actual grids, and
        % generate grids if yes
        if isnumeric(cfg.xgrids),
            xgrids = cell(p, 1);
            for i = 1:p, xgrids{i} = cfg.gridfun(cfg.xgrids(i), maxx(i)); end;
        else
            xgrids = cfg.xgrids;
        end;
        if isnumeric(cfg.ugrids),
            ugrids = cell(q, 1);
            for i = 1:q, ugrids{i} = cfg.gridfun(cfg.ugrids(i), maxu(i)); end;
        else
            ugrids = cfg.ugrids;
        end;
        % cross-product to get all points
        XU = flat({xgrids{:}, ugrids{:}});
        % initialize # of samples, pick up X and U samples
        N = size(XU, 2);
        X = XU(1:p, :);
        U = XU(p+1:p+q, :);
        % initialize and generate corresponding next states and rewards
        Xp = 0.*X; R = zeros(1, N); T = zeros(1, N);
        for i = 1:N,
            [Xp(:, i), R(i), T(i)] = fun(model, X(:, i), U(:, i));
        end;
                
    otherwise,
        error(['Unknown sample generation method [' method ']']);
end;

% finalize
samples = varstostruct('N', 'X', 'U', 'Xp', 'R', 'T');
overwrite = 0;   % make sure cfg fields don't overwrite computed data
samples = copyfields(cfg, samples, [], overwrite);

end % GENERATE_SAMPLES returning SAMPLES -------------------


% Obsolete sample generation methods:
%         
%     case 'expdisc',
%         mux = cfg.sigmax(:);
%         i = 1;
%         while i <= N,
%             X(:, i) = exprnd(mux);
%             if any(abs(X(:, i)) > maxx), continue; end; % reject sample
%             % switch signs according to a uniform discrete distr
%             s = unidrnd(2, 2, 1); X(s==1,i) = -X(s==1,i);
%             U(:, i) = Uflat(:, unidrnd(M));
%             [Xp(:, i), R(i)] = fun(model, X(:, i), U(:, i));
%             i = i + 1;
%         end;
% 
