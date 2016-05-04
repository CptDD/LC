function varargout = approxqlearn(cfg)
% Approximate Q-learning with a linear parametrization and gradient-based updates
%   VARARGOUT = APPROXQLEARN(CFG)
% Inputs:
%   CFG             - structure with fields as commented in the code below
%           can also be given as a string, see he str2cfg
% Outputs:
%   THETA           - in run mode. Found parameter vector.
%   HIST, FIGH      - in replay mode -- NOT IMPLEMENTED. HIST is the replay history.
%           FIGH contains figure handles if figures were created 
%           and not closed; and is an empty matrix if all the figures were closed.

% TODO visualization code not tested

if nargin < 1, cfg = struct; end;

% -----------------------------------------------
% Process configuration structure

% default config
% script config
CFG.model_params = {};              % extra parameters for problem calling in 'model' mode
CFG.run = 0;                        % run learning
CFG.replay = 0;                     % replay learned policy
CFG.evol = 0;                       % show evolution
CFG.traj = 0;                       % show some trajectories
CFG.problem = '';                   % what problem to solve
CFG.datadir = '';                   % data dir
CFG.datafile = '';                  % save data to file
CFG.approx = [];                    % approximator object (or config for approx)
CFG.gamma = [];                     % discount factor
CFG.alpha = .1;                     % learning rate
CFG.alphadecay = 1;                 % per trial
CFG.explor = .9;                    % exploration rate
CFG.explordecay = .9;               % exploration decay (per trial)
CFG.lambda = 0;                     % eligibility trace parameter
CFG.resetetronexplore = 1;          % whether to reset trace on exploration
CFG.eps = .01;                      % threshold for convergence
CFG.maxtime = 100;                  % max length of learning experiment (in seconds)
CFG.trialtime = 10;                 % max length of a trial (in seconds)
CFG.reset = 'rand';                 % trial reset
CFG.randseed = [];                  % set the random seed (for repeatable results)
% replay, etc. config
CFG.x0 = [];                        % initial state for replay (otherwise the problem default or zeros)
CFG.tend = 10;                      % end time for replay
CFG.trialskip = 10;                 % for traj/evol, display one out of this many trials

% output config
CFG.trialsave = 50;                 % save once this many trials
CFG.plottarget = 'screen';          % 'screen', '', 'latex', or 'beamer'. If 'screen' figures will not be closed
CFG.savedir = '';
CFG.savefig = '';
% display config
CFG.silent = 0;                     % run in silent mode (e.g., during replay)
CFG.verb = 5;                       % verbosity: the higher, the more detailed the messages displayed
CFG.visualize = 0;                  % visualization level (0 = none, 1 = trial-level, 2 = step-level)
CFG.viscfg = struct;                % visualization config options
CFG.trialdisp = 10;                 % display console feedback once this many trials

% List of fields that define the problem and therefore may NOT be overwritten on load
KEEPFIELDS = {'problem', 'gamma'};

% Install function defaults for everything else
cfg = parseconfig(cfg, CFG);

cfg.init = cfg.run;

% % If running, check presence of data file, warn on possible overwrite
% if cfg.run && exist([cfg.datafile '.mat'], 'file'),
%     reply = input(['File [' cfg.datafile '.mat] already exists (possible overwrite). Continue? Y/N [N]: '], 's');
%     if isempty(reply) || reply == 'N', return; end;
% end;       

if ~cfg.init,        % load data file, making sure that cfg is not overwritten
    cfg1 = cfg; kf = KEEPFIELDS;
    load(cfg.datafile);
    % Overwrite problem-defining fields from loaded config, keep the rest as in the (current) cfg1;
    % the result becomes the new config
    cfg = copyfields(cfg, cfg1, kf);
    KEEPFIELDS = kf; clear cfg1 kf;
    dispx(['Data loaded from [' cfg.datafile '].'], cfg.verb, 1);
    approx = revise_approx(approx);
end;

% Echo config
dispx('Approximate Q-learning called with the following configuration:', cfg.verb, 1);
dispx(cfg, cfg.verb, 1);

% get environment (Matlab, hardware) info
cfg.envinfo = getenvx;

% -----------------------------------------------
% Create model, find dimensionality data
if cfg.init,
    model = feval(cfg.problem, 'model', cfg.model_params{:});
    Ts = model.Ts;
    % heuristic check if we already have an approx object
    if isstruct(cfg.approx) && isfield(cfg.approx, 'N') && isfield(cfg.approx, 'phi') ...
            && isfield(cfg.approx, 'q') && isfield(cfg.approx, 'h'),
            % nothing to do
            dispx('Approx object supplied', cfg.verb, 3);
    else    % create approximator using cfg.approx as the config
        cfg.approx = create_approx(model, cfg.approx);
    end;
    cfg.approx = revise_approx(cfg.approx);
    approx = cfg.approx;
    N = approx.N; p = model.p; q = model.q;
    if approx.disc,        % initialize action grids and their size
        discU = cfg.approx.U;
        flatU = flat(discU); nU = size(flatU, 2);
        % discUn = zeros(length(discU)); for i=1:length(discU), discUn(i) = length(discU{i}); end;        
    end;
    if ~approx.extendedsyntax,
        error('Approximator that supports extended syntax required');
    end;
end;

% -----------------------------------------------
% Q-iteration
if cfg.run,
    
    % derive Ktrial and maxtrials
    maxtrials = cfg.maxtime / cfg.trialtime;
    Ktrial = cfg.trialtime / model.Ts;
    
    % init Q-function, elig trace, histories etc.
    X = NaN(p, Ktrial+1, maxtrials);
    U = NaN(q, Ktrial+1, maxtrials);
    R = NaN(1, Ktrial+1, maxtrials);
    theta = zeros(N, 1);
    E = zeros(N, 1);                    % eligibility trace
    thetah = cell(maxtrials+1, 1);  
    thetah{1} = theta;                  % save theta_0 on the stats
    deltah = NaN(maxtrials+1, 1);
    
    k = 1;                              % total step counter
    trial = 1;                          % trial counter
    timestat.run = 0;
    explor = cfg.explor;
    alpha = cfg.alpha;
    
    if ~isempty(cfg.randseed), rand('twister', cfg.randseed); end;
    
    % init visualization config if needed
    if cfg.visualize,
        vcfg = cfg.viscfg;
        vcfg.gview = [];
        % here, one could visualize initial state of the algorithm
        % but currently there is no meaningful initial state, since
        % we just visuzalize steps
        % [figh vcfg.gview] = feval(model.visualizefun, vcfg);
    end;
    
     % -----------------------------------------------
    % Perform algorithm
    if cfg.lambda > 0, dispx('Performing approximate Q(lambda)-learning...', cfg.verb, 0);
    else dispx('Performing approximate Q-learning...', cfg.verb, 0);
    end;

    t = cputime;
    conv = 0;
    while trial <= maxtrials && ~conv,       % main loop

        % initialize trial
        % reset to initial state
        X(:, 1, trial) = feval(cfg.problem, 'reset', model, cfg.reset);
        % step counter, terminal flag
        ktrial = 1; % term = 0;
        % reset elig trace
        E(:) = 0;

        timestat.run = timestat.run + (cputime - t);
        % visualize initial state
        if cfg.visualize >= 2,
            vcfg.approxqlearnstep = 1;
            vcfg.approxqlearntrial = 0;
            vcfg.k = ktrial - 1;
            vcfg.ktotal = ktrial - 1 + Ktrial * (trial - 1); % this only works in continuing tasks
            vcfg.trial = trial;
            [figh vcfg.gview] = feval(model.visualizefun, vcfg);
        end;
        t = cputime;
        
        while ktrial <= Ktrial, % && ~term,        
            
            % choose action according to current state
            exploring = rand < explor;
            % choose control action
            if exploring,  % explore, choose uniform random action
                if approx.disc,    % flatU is a set of discrete actions, each action on a column
                    U(:, ktrial, trial) = flatU(:, unidrnd(nU));
                else        % U is continuous, assumed symmetrical w/ bounds in maxu
                    U(:, ktrial, trial) = (2*rand(q, 1) - 1) .* model.maxu;
                end;
            else            % exploit using policy greedy in the current theta
                U(:, ktrial, trial) = approx.h(approx, theta, X(:, ktrial, trial));
            end;
            
            % apply the action
            [X(:, ktrial+1, trial), R(1, ktrial+1, trial), term] = ...
                feval(model.fun, model, X(:, ktrial, trial), U(:, ktrial, trial));
            if term, error('APPROXQLEARN: terminal state support not implemented'); end;
            
            % process elig trace if using it
            if cfg.lambda > 0,
                if exploring && cfg.resetetronexplore,   
                    E(:) = 0;
                else
                    E = (cfg.gamma * cfg.lambda) .* E;
                end;
            end;
            
            % compute temporal difference and update Q-function parameters
            [Qxu, phixu] = approx.q(approx, theta, X(:, ktrial, trial), U(:, ktrial, trial));
            [up, Qstarxpup] = approx.h(approx, theta, X(:, ktrial+1, trial));
            tempdiff = R(1, ktrial+1, trial) + cfg.gamma * Qstarxpup - Qxu;
            if cfg.lambda > 0,  % use elig trace
                E = E + phixu;
                theta = theta + (alpha * tempdiff) .* E;
            else                % no trace
                theta = theta + (alpha * tempdiff) .* phixu;
            end;
                
            
            % update stats
            timestat.run = timestat.run + (cputime - t);
            
            % visualization
            if cfg.visualize >= 2,
                vcfg.approxqlearnstep = 1;
                vcfg.approxqlearntrial = 0;
                vcfg.k = ktrial;
                vcfg.trial = trial;
                vcfg.ktotal = ktrial - 1 + Ktrial * (trial - 1); % this only works in continuing tasks
                [figh vcfg.gview] = feval(model.visualizefun, vcfg);
            end;
            
            % (in the future, if alpha must be annealed, it should be done here)
            
            t = cputime; 
            ktrial = ktrial + 1;
            k = k + 1;
        end;
      
            
        % store Q-function on history
        thetah{trial+1} = theta;
        % compute max absolute difference
        deltah(trial+1) = max(max(abs(theta - thetah{trial})));
        conv = deltah(trial+1) < cfg.eps;

        % update execution time -- before starting to visualize, output to console etc.
        timestat.run = timestat.run + (cputime - t);
        
        % visualization
        if cfg.visualize >= 1,
            vcfg.approxqlearntrial = 1;
            vcfg.approxqlearnstep = 0;
            vcfg.trial = trial;
            vcfg.ktotal = Ktrial * trial; % this only works in continuing tasks
            [figh vcfg.gview] = feval(model.visualizefun, vcfg);
        end;
        
        % console feedback of algorithm progress
        if ~mod(trial, cfg.trialdisp), 
            dispx(sprintf('Trial #%d completed, delta=%.3f', trial, deltah(trial+1)), cfg.verb, 2);
        end;
        % data backup
        if ~mod(trial, cfg.trialsave),
            save(cfg.datafile);
            dispx(sprintf('Data at k=%d (t=%f) saved to [%s].', (k-1), (k-1)*Ts, cfg.datafile), cfg.verb, 1);
        end;
        
        % start counting time again, increment trial counter
        t = cputime;
        trial = trial + 1;
        % anneal exploration
        explor = explor * cfg.explordecay;
        % anneal learning rate
        alpha = alpha * cfg.alphadecay;
    end;        % while not converged and allowed more iterations

    if conv,	dispx('Convergence detected. Algorithm stopped.', cfg.verb, 0);
    else        dispx(sprintf('maxtime=%f exhausted. Algorithm stopped.', cfg.maxtime), cfg.verb, 0);
    end;
    
    % finalize visualizer
    if cfg.visualize,
        vcfg.approxqlearntrial = 0;
        vcfg.approxqlearnstep = 0;
        vcfg.finalize = 1;
        [figh vcfg.gview] = feval(model.visualizefun, vcfg);
    end;
    
    % output parameter vector
    varargout = {theta};
end;


% Replay using learned policy
if cfg.replay,

    % initial state
    if ~isempty(cfg.x0), 
        if ischar(cfg.x0) && ~isempty(cfg.problem),
            % use the model function to get a string-named initial state
            x0 = feval(cfg.problem, 'x0', cfg.x0);
        else
            x0 = cfg.x0(:);         % specified initial state
        end;
    else    % try getting default initial state
        try         x0 = feval(cfg.problem, 'x0');
        catch       x0 = zeros(model.p, 1);    % zeros
        end;
    end;         
    dispx(['Controlling from x0=' num2str(reshape(x0, 1, [])) ], cfg.verb, 0);

    % init history
    t = 0 : model.Ts : cfg.tend;
    Ns = length(t)-1;       % number of samples / time instances at which control is applied
    x = zeros(model.p, length(t)); x(:, 1) = x0;
    u = zeros(model.q, length(t)); u(:, end) = NaN;
    r = zeros(1, length(t)); r(1) = NaN;

    % steps loop
    for k = 1:Ns,
        % compute action
        u(:, k) = approx.h(approx, theta, x(:, k));
        % apply to system
        [x(:, k+1) r(k+1) terminal] = feval(model.fun, model, x(:, k), u(:, k));
        if terminal, Ns = k; u(:, k+1) = NaN; break; end;      % entered terminal state
    end;
 
    % plot history & optionally save figures
    hist.t = t(1:Ns+1); hist.x = x(:, 1:Ns+1); hist.u = u(:, 1:Ns+1); hist.r = r(1:Ns+1);
    hist.R = discreturn(cfg, hist.r, Ns, terminal);
    if ~cfg.silent,
        % NOTE if plotfun produces nothing, no plot is created...
        if isfield(model, 'plotfun'),
            mpfigh = feval(model.plotfun, hist);
            if ~isempty(mpfigh), figh = mpfigh; end;
        else
            figh = plothistory(hist);
        end;
        setfigprop(cfg);
        % save if requested
        saveplot(figh, [cfg.savedir cfg.savefig], cfg.plottarget);
    end;
end;        % IF replay

% Plot some trajectories
if cfg.traj,
    figh = figure;
    % k'screen' refers to coordinates on screen (time lapse)
    % k, trajk refers to (simulated) real-time coordinates
    trajkscreen = []; xticklabels = {};
    if isscalar(cfg.trialskip),     trials = 1:cfg.trialskip:size(X, 3);
    else                            trials = cfg.trialskip;
    end;
    for i=1:length(trials),
        trial = trials(i);
        k = (trial-1)*Ktrial; kscreen = (i-1)*Ktrial;
        Ktrial = find(~isnan(U(1, :, trial)), 1, 'last');  % not counting the time step AFTER the last control
        for ip = 1:p,
            subplot(p+q+1, 1, ip); hold on;
            plot((kscreen + (0:Ktrial))*Ts, X(ip, 1:Ktrial+1, trial), 'k-');
        end;
        for iq = 1:q,
            subplot(p+q+1, 1, p+iq); hold on;
            plot((kscreen + (0:Ktrial-1))*Ts, U(iq, 1:Ktrial, trial), 'k-');
        end;
        subplot(p+q+1, 1, p+q+1); hold on;
        plot((kscreen + (1:Ktrial))*Ts, R(1, 2:Ktrial+1, trial), 'k-');
        trajkscreen(i) = kscreen;
        xticklabels{i} = sprintf('%.1f', k * Ts);
    end;
    % finally add some markers in-between
    for ip = 1:p+q+1,
        subplot(p+q+1, 1, ip);
        y = get(gca, 'YLim');
        for i = 1:length(trials), line(trajkscreen(i)*Ts + [0 0], y, 'Color', 'r'); end;
        if ip <= p, 
            xlim([0 length(trials)*Ktrial] * Ts);
            ylabel(['x_{' num2str(ip) '}']); 
            set(gca, 'XTick', trajkscreen * Ts);
            set(gca, 'XTickLabel', xticklabels);
        elseif ip <= p+q, 
            xlim([0 length(trials)*Ktrial] * Ts);
            ylabel(['u_{' num2str(ip-p) '}']);
            set(gca, 'XTick', trajkscreen * Ts);
            set(gca, 'XTickLabel', xticklabels);
        else % last plot, also add time lables
            xlim([0 length(trials)*Ktrial] * Ts);
            ylabel('r'); xlabel('Time Lapse');
            set(gca, 'XTick', trajkscreen * Ts);
            set(gca, 'XTickLabel', xticklabels);
        end;
    end;
    set(gcf, 'NumberTitle', 'off', 'Name', cfg.datafile);
    
    varargout = {figh};
end;

if cfg.evol,
    if exist('cleanedup', 'var') && cleanedup >= 2,
        dispx('Cannot replay evolution, hard cleanup was performed on the datafile.', cfg.verb, 0);
    else
        figh = figure; % colormap(sty.cm);
        for trial = 1:cfg.trialskip:length(thetah),
            figcfg.figname = sprintf('Trial=%d [%s]', trial, cfg.datafile);
            setfigprop(figcfg);
            subplot(221); cla;
            approx.plotv(approx, thetah{trial}, 'npoints=30'); % title(labels.V);
            subplot(222); cla;
            approx.ploth(approx, thetah{trial}, 'npoints=30'); % title(labels.h);
            subplot(2, 2, [3 4]); cla;
            semilogy(deltah(1:trial), 'k'); % xlabel(labels.time); ylabel(labels.delta{:});
            if trial < length(thetah), pause; end;
        end;
        % save if requested
        saveplot(figh, [cfg.savedir cfg.savefig], cfg.plottarget);
    end;
end;

% -----------------------------------------------
% Backup data

if cfg.run && ~isempty(cfg.datafile),
    % save into the same directory as the problem
    if isempty(cfg.datadir),
        datadir = fileparts(which(cfg.problem));
    else
        datadir = cfg.datadir;
        if (datadir(end) == '\'), datadir = datadir(1:end-1); end;
    end;
    cfg.datafile = [datadir '\' cfg.datafile];
    save(cfg.datafile);
    dispx(['Data was saved to [' cfg.datafile '].'], cfg.verb, 1);
end;


% END qi() RETURNING varargout =================================================================
