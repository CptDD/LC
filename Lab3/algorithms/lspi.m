function varargout = lspi(cfg)
% Least-squares policy iteration.
%   [THETA, TIMESTAT] = LSPI(CFG)   - in 'run', 'resume' modes
%   [HIST, FIGH] = LSPI(CFG)        - in 'replay' mode
%   FIGH = LSPI(CFG)                - in 'sol' or 'evol' modes
% An implementation of least-squares policy iteration [1]. Requires an approximator structure or
% configuration on the field 'approx', see create_approx for details. Similarly, requires a samples
% structure or configuration on 'samples', see generate_samples.
% Checks for convergence (difference between consecutive parameters below eps), for oscillation of
% the solution, as well as for noninvertibility of matrix A.
% If the approximator supports so-called "index optimization" (see e.g., the rbfdisc approximator),
% the algorithm will exploit it. Precomputes as much as possible of the A and b matrices beforehand.
%
% The main way of outputting data is a datafile, which will be saved in 'run' and 'resume' modes. 
% The function outputs are provided just for convenience.
%
% Inputs:
%   CFG             - structure with fields as commented in the code below
%           can also be given as a string, see str2cfg
% 
% Outputs:
%       THETA       - the computed parameter vector
%       HIST        - the replay history (trajectories)
%       FIGH        - handles to the figures created
%
% [1] Lagoudakis, M. G. & Parr, R. "Least-Squares Policy Iteration"
%   Journal of Machine Learning Research, 2003, 4, 1107-1149

% Author: Lucian Busoniu
% Version history:
%   1.0     - added recursive inverse code
%   2.0     - removed recursive inverse code, added initial policy code
%   2.01    - removed if storephi test from inner sample loop, for speed
%   3.0     - moved policy-indep parts (b, A0) out of the iterations loop
%             implemented index (offset) optimization; restored storephi test
%   3.1     - added detailed execution time statistics
%
% Detailed execution time statistics are stored in variable "timestat". Fields:
%   qsamples        - scalar, time to generate the policy eval samples
%   precomputephi   - scalar, time to precompute BF values (if precomputed)
%   precomputeAb    - scalar, time to precompute fixed part of A, and b (if precomputed)
%   init            - scalar, aggregate initialization time (sum of all of the above)
%   iter_computeAb  - vector, time/iter to compute A and b 
%   iter_qsolve     - vector, time/iter to solve A theta = b
%   iter_peval      - vector, time/iter to perform policy eval (iter_computeAb + qlsqr)
%   iter_convtest   - vector, time/iter to test convergence and oscillation
%   iter_run        - vector, total runtime/iter (iter_peval + iter_convtest)
%   run             - scalar, aggregate runtime (sum of all iter_run)
% For compatibility reasons, a variable called "trun" is also kept, equal
% to timestat.run

% WARNING 'resume' mode not thoroughly tested; use at own risk

if nargin < 1, cfg = struct(); end;

% ==== DECLARE CONFIGURATION DEFAULTS ====
% function config
CFG.run = 0;                        % run learning
CFG.resume = 0;                     % resume learning
CFG.replay = 0;                     % replay learned policy
CFG.sol = 0;                        % plot solution and convergence stats
CFG.evol = 0;                       % plot evolution statistics
CFG.datafile = 'lspidata';          % save data to file
CFG.datadir = [];                   % data dir
% main algorithm config
CFG.problem = '';                   % what problem to solve
CFG.approx = [];                    % approximator object (or config for approx)
CFG.samples = [];                   % collection of samples object (or config for generate_samples)
CFG.gamma = [];                     % discount factor
CFG.maxiter = 100;                  % maximum # of iterations
CFG.term = 'ignore';                % how to handle terminal states ('ignore' or 'zero' the next-state Q-values/BFs)
CFG.method = 'inv';                 % parameter computation method: currently only 'inv' supported
CFG.invdelta = 0;                   % use for inverse computation
CFG.recinvdelta = 1e-3;             % currently not used; for recursive inverse computation (has to be nonzero)
CFG.indexoptimized = 1;             % use index-optimized implementation when available (set to 0 to override)
CFG.loadsamples = [];               % load samples from this file
CFG.h0 = [];                        % start with this initial policy
CFG.h0args = {};                    % arguments for the initial policy
% convergence and oscillation checks
CFG.eps = 0.01;                     % convergence threshold
CFG.maxper = 10;                    % osc: maximum period to check (# iter)
CFG.nper = 1;                       % osc: number of periods to check
CFG.epsper = 0.01;                  % osc: maximum abs diff between two par vectors in diff periods
% replay config
CFG.x0 = [];                        % initial state for replay (otherwise the problem default or zeros)
CFG.tend = 30;                      % end time for replay
% stats config
CFG.computecondA = 1;               % whether to compute condition number of A
CFG.storeab = 0;                    % whether to store A and b histories
% display config
CFG.visualize = 0;                  % visualization level (0 = none, 1 = iteration-level)
CFG.viscfg = struct;                % visualization config options
CFG.verb = 3;                       % verbosity: the higher, the more detailed the messages displayed
CFG.iterdisp = 1;                   % feedback after every iterdisp iterations
CFG.itervis = 1;                    % visualize after every itervis iterations
CFG.itersave = 3;                   % save after each itersave iterations
CFG.sampledisp = .05;               % display progress after this % of samples processed
CFG.silent = 0;                     % suppress all output
% figure config
CFG = setfigprop(CFG, 'addfields');
CFG.miscinfo = '';                  % field for miscellaneous information

% Early defaults (initialized before calling problem defaults)
ECFG.model_params = {};             % parameters for problem calling in 'model' mode
ECFG.lspi_params = {};              % parameters for problem calling in 'lspi' mode

% List of fields that define the problem
KEEPFIELDS = {'problem', 'gamma', 'approx', 'samples', 'usesparse'};

% ==== PARSE AND PROCESS CONFIG ====
cfg = parseconfig(cfg, CFG, ECFG, 'lspi');
% get environment (Matlab, hardware) info
cfg.envinfo = getenvx;

% If running, check presence of data file, warn on possible overwrite
% disabled for lab purposes
% if cfg.run && exist([cfg.datafile '.mat'], 'file'),
%     reply = input(['File [' cfg.datafile '.mat] already exists (possible overwrite). Continue? Y/N [N]: '], 's');
%     if isempty(reply) || reply == 'N', return; end;
% end;       
% determine if initialization needed; if not, need to load: check if datafile exists
cfg.init = ~(cfg.resume || cfg.replay || cfg.sol || cfg.evol);
if ~cfg.init && ~exist([cfg.datafile '.mat'], 'file'),
    error(['File [' cfg.datafile '.mat] does not exist. Terminating.']);
end;

% ==== IF NOT INITIALIZING: NEED TO LOAD DATA ====
if ~cfg.init,        % load data file, making sure that cfg and KEEPFIELDS is not overwritten
    % optimize the loading time: only load large variables when needed
    dfv = who('-file', cfg.datafile);
    if cfg.sol || cfg.replay || cfg.evol,
        % no need for the following auxiliary variables (helps with the loading time):
        dfv = rmstring(dfv, 'A', 'A0', 'b', 'PHI', 'Xs', 'samples', 'Xs', 'Us', 'Xps', 'Rs');
        % when not evol plot, the history of params is also not needed
        if ~cfg.evol, dfv = rmstring(dfv, 'thetah'); end;
    end;
    cfg1 = cfg; kf = KEEPFIELDS;
    load(cfg.datafile, dfv{:});
    % Overwrite problem-defining fields from loaded config, keep the rest as in the (current) cfg1;
    % the result becomes the new config
    cfg = copyfields(cfg, cfg1, kf);
    KEEPFIELDS = kf; clear cfg1 kf;
    dispx(['LSPI: data loaded from [' cfg.datafile '].'], cfg.verb, 1);
    cfg.approx = revise_approx(cfg.approx);     % make sure approx is in latest format
    approx = revise_approx(approx);    % for replay, etc functions that assume approx is initialized
    cfg.samples = revise_samples(cfg.samples);  % the same for samples
    if exist('htheta', 'var'), 
        reply = input(['The algorithm used to produce this data was LSPIH.' 13 ....
            'Do you want to continue processing with LSPI? Y/N [N]: '], 's');
        if isempty(reply) || reply == 'N', return; end;
    end;
end;

if ~strcmp(cfg.method, 'inv'), error('Currently only normal inversion (INV) method supported.'); end;

% Echo config
dispx('LSPI will run with the following configuration:', cfg.verb, 1);
dispx(cfg, cfg.verb, 1, 'cfg', 'config');


% ==== IF INITIALIZING: CREATE MODEL, AND IF NEEDED: APPROXIMATOR, SAMPLES ====
if cfg.init,
    model = feval(cfg.problem, 'model', cfg.model_params{:});
    timestat = struct;
    timestat.init = 0;
    % heuristic check if we already have an approx object
    if isstruct(cfg.approx) && isfield(cfg.approx, 'N') && isfield(cfg.approx, 'phi') ...
            && isfield(cfg.approx, 'q') && isfield(cfg.approx, 'h'),
            % nothing to do
            dispx('Approx object supplied', cfg.verb, 3);
    else    % create approximator using cfg.approx as the config
        cfg.approx = create_approx(model, cfg.approx);
    end;
    % samples object
    if ~isempty(cfg.loadsamples) && exist([cfg.loadsamples '.mat'], 'file'),
        % try loading samples -- performs no checking whether the same type & number of
        % samples is required
        loadwithprefix(cfg.loadsamples, 's_', 'cfg');
        cfg.samples = s_cfg.samples;
        clear s_cfg;
        dispx(['Samples object loaded from [' cfg.loadsamples ']'], cfg.verb, 1);
    else
        % heuristic check if we already have a samples collection supplied
        if isstruct(cfg.samples) && isfield(cfg.samples, 'N') && isfield(cfg.samples, 'X') ....
                && isfield(cfg.samples, 'U') && isfield(cfg.samples, 'Xp') ...
                && isfield(cfg.samples, 'R'),
                dispx('Samples object supplied', cfg.verb, 3);
            % nothing to do
        else
            % generate samples using cfg.samples as the config
            % sample generation is counted in the initialization time
            dispx('Generating samples...', cfg.verb, 0);
            tmark_init = cputime;
            cfg.samples = generate_samples(model, cfg.samples, cfg.approx);
            timestat.qsamples = cputime - tmark_init; clear tmark_init;
            timestat.init = timestat.init + timestat.qsamples;
            dispx([8 ' done.'], cfg.verb, 0);
        end;
    end;
    % initial policy
    if ~isempty(cfg.h0),
        % determine the order of the possibly dynamic controller
        try     [cx0, cord] = feval(cfg.h0, 'init', cfg.h0args{:});
        catch   cord = 0;   % compatibility for old-style static policies
        end;
        if cord > 0, error('Cannot control non-trajectory samples with a dynamical controller'); end;    
    end;
end;

% cfg.approx
% cfg.approx.Uflat
% cfg.samples

% ==== RUN LSPI ====
if cfg.run || cfg.resume,
    
    % shorthand variables
    approx = cfg.approx;
    N = approx.N;
    Ns = cfg.samples.N;
    Xs = cfg.samples.X; Us = cfg.samples.U; Xps = cfg.samples.Xp; Rs = cfg.samples.R; Ts = cfg.samples.T;
    % use index-optimized implementation if configured and approximator supports it
    cfg.indexoptimized = cfg.indexoptimized && isfield(approx, 'indphi');
    indexoptimized = cfg.indexoptimized;        % shorthand
    if indexoptimized, Nip = approx.Nindphi; end;   % shorthand

    % Initialize if not resuming
    if cfg.init,
        % History
        if cfg.storeab,
            Ah = cell(cfg.maxiter, 1);
            bh = cell(cfg.maxiter, 1);
        end;
        if cfg.computecondA,
            condA = nan(cfg.maxiter, 1);  % condition number history for A
        end;
        thetah = cell(cfg.maxiter+1, 1);
        deltah = nan(cfg.maxiter+1, 1);      % deltah(1) will always remain NaN

        % use sparse A and PHI if the approx type recommends it
        if isfield(approx, 'sparse') && approx.sparse, 
            cfg.usesparse = 1; zeromatrix = @sparse;
        else
            cfg.usesparse = 0; zeromatrix = @zeros;
        end;
        % if worst-case storage space for pre-computed BF values of all the (x, u) samples
        % is reasonable, pre-init; for now, pre-init is always on (hardcoded)
        storephi = 1; %(N * Ns * 8) <= 500e6;   % 500 MB with double (8 bytes) storage
        
        k = 1;                  % iteration index
        timestat.run = 0;       % execution time
        theta = zeros(N, 1);    % initialize parameter vector to zeroes
        thetah{1} = theta;      % put it on the history
    end;        % initialization IF

    if ~storephi,
        dispx('WARNING! BF values are not pre-computed. Algorithm will run slowly.', cfg.verb, 0);
    end;
    if any(Ts) && cfg.term(1) == 'i', 
        dispx('WARNING! Terminal states present and will be ignored.', cfg.verb, 0);
    end;
    
    % PHI is computed when initializing, or when loading a file that has been cleaned up
    if storephi && (cfg.init || ~exist('PHI', 'var')),
        if cfg.init,    
            dispx('Pre-computing BF values for the samples...', cfg.verb, 0);
            tmark_init = cputime;
        else
            dispx('Re-computing BF values for the samples...', cfg.verb, 0);
        end;
        if indexoptimized,
            PHI = zeromatrix(Nip, Ns);   % indices
            OFF = zeros(1, Ns);          % offsets
            for i = 1:Ns, [OFF(i), PHI(:, i)] = approx.indphi(approx, Xs(:, i), Us(:, i)); end;
        else
            PHI = zeromatrix(N, Ns);
            for i = 1:Ns, PHI(:, i) = approx.phi(approx, Xs(:, i), Us(:, i)); end;
        end;
        % Only the first computation of BF values is counted in
        % the init time (otherwise tinit would grow with each recomputation)
        if cfg.init, 
            timestat.precomputephi = cputime - tmark_init; clear tmark_init;
            timestat.init = timestat.init + timestat.precomputephi; 
        end;
        dispx([8 ' done.'], cfg.verb, 0);
    end;
    
    dispx('Performing LSPI...', cfg.verb, 0);
    
    % init visualization config if needed
    if cfg.visualize,
        vcfg = cfg.viscfg;
        vcfg.gview = [];
        vcfg.lspiter = 1;
        % visualize initial state of the algorithm
        vcfg.ell = 0;
        [figh vcfg.gview] = feval(model.visualizefun, vcfg);
    end;        
    
    
    % pre-compute A0 and b when initializing, or when loading a file that has been cleaned up
    if cfg.init || ~exist('b', 'var') || ~exist('A0', 'var'),
        dispx('Pre-computing b and fixed part of A, samples processed: 00%', cfg.verb, 1);
        tmark_init = cputime;
        progs = cfg.sampledisp;         
        % init A0 and b
        if cfg.invdelta > 0,    % invdelta on the diagonal
            if cfg.usesparse,   A0 = sparse(1:N, 1:N, cfg.invdelta, N, N);
            else                A0 = eye(N, N) .* cfg.invdelta;
            end;
        else                A0 = zeromatrix(N, N);  % just zeros
        end;
        b = zeros(N, 1);        % b needs not be sparse
        for i = 1:Ns,
            % retrieve or compute phi(x, u) (optimized or not); update A0 and b
            if indexoptimized,
                if storephi,    off = OFF(i); phixu = PHI(:, i);
                else            [off, phixu] = approx.indphi(approx, Xs(:, i), Us(:, i));
                end;            
                A0(off+1:off+Nip, off+1:off+Nip) = A0(off+1:off+Nip, off+1:off+Nip) + phixu * phixu';
                b(off+1:off+Nip) = b(off+1:off+Nip) + phixu * Rs(i);
            else
                if storephi,    phixu = PHI(:, i);
                else            phixu = approx.phi(approx, Xs(:, i), Us(:, i));
                end;            
                A0 = A0 + phixu * phixu';
                b = b + phixu * Rs(i);
            end;
            if i/Ns >= progs,    % display progress inline
                dispx([8 8 8 8 sprintf('%2d%%', fix(i/Ns*100))], cfg.verb, 1);
                progs = progs + cfg.sampledisp;
            end;
        end;
        timestat.precomputeAb = cputime - tmark_init; clear tmark_init;
        timestat.init = timestat.init + timestat.precomputeAb; 
        dispx([8 8 8 8 8 '100%. Done.'], cfg.verb, 3);
        save(cfg.datafile);
        dispx(['Init data saved to [' cfg.datafile '].'], cfg.verb, 1);
    end;
    
    % check if should use an initial policy
    useh0 = ~isempty(cfg.h0);
    if useh0;
        disc = any(strcmp(approx.type, {'rbfdisc', 'triang', 'polydisc'}));
        if disc,        % initialize action grids and their size
            U = approx.U; 
            Un = zeros(length(U)); for i=1:length(U), Un(i) = length(U{i}); end;        
        end;
    end;
    
    % Main policy iteration loop
    conv = 0; Anoninv = 0;
    while ~conv && ~Anoninv && k <= cfg.maxiter,
        tmark = cputime;
        
        % clear policy-dependent part of A
        A = A0;
        
        % Loop through samples, updating A
        progs = cfg.sampledisp; dispx(sprintf('LSTD-Q k=%d iter, samples processed: %2d%%', k, 0), cfg.verb, 3);
        for i = 1:Ns,
            % if terminal state & configured to always set terminal state values to zero,
            % then phixpup = 0; or, equivalently, we skip the update of A entirely
            if Ts(i) && cfg.term(1) == 'z', continue; end;
            % find out the action in x'
            if k == 1 && useh0,         % use an initial policy
                up = cfg.h0(Xps(:, i), [], cfg.h0args{:});  % [] = undef time arg
                if disc, up = quantize(up, U, Un); end;     % quantize action if discrete-action approx
            else                        % use the current poolicy
                up = approx.h(approx, theta, Xps(:, i));
            end;
            
            % retrieve or compute phi (optimized or not), compute phi(x', h(x'), and update A
            if indexoptimized,
                if storephi,    off = OFF(i); phixu = PHI(:, i);
                else            [off, phixu] = approx.indphi(approx, Xs(:, i), Us(:, i));
                end;            
                [offp, phixpup] = approx.indphi(approx, Xps(:, i), up);
                A(off+1:off+Nip, offp+1:offp+Nip) = A(off+1:off+Nip, offp+1:offp+Nip) ...
                    - cfg.gamma * phixu * phixpup';
            else
                if storephi,    phixu = PHI(:, i);
                else            phixu = approx.phi(approx, Xs(:, i), Us(:, i));
                end;            
                phixpup = approx.phi(approx, Xps(:, i), up);
                A = A - cfg.gamma * phixu * phixpup';
            end;
            
            if i/Ns >= progs,    % display progress inline
                dispx([8 8 8 8 sprintf('%2d%%', fix(i/Ns*100))], cfg.verb, 3);
                progs = progs + cfg.sampledisp;
            end;
        end;
        dispx([8 8 8 8 8 '100%. Solving LSQR...'], cfg.verb, 3);
        timestat.iter_computeAb(k) = cputime - tmark; tmark = cputime;
        lastwarn('');
        % theta = A \ b;    
        % replaced with version w/ normalization on 2009-09-10
        theta = (A./Ns) \ (b./Ns);
        if ~isempty(lastwarn), Anoninv = 1; end;    % set non-invertibility flag 
        dispx([8 ' done.'], cfg.verb, 3);
        timestat.iter_qsolve(k) = cputime - tmark; tmark = cputime; 
        
        deltah(k+1) = norm(theta - thetah{k}, 2);
        thetah{k+1} = theta;                        % add theta to history
        
        % check convergence condition
        conv = deltah(k+1) <= cfg.eps;
        
        % check for oscillations
        % NOTE: for many existing experiments osc test was not included!
        per = 2; osc = 0;   % don't test for period = 1, that's taken care of by the simple convergence test
        % try periods up to maxper, as long as oscillation wasn't detected yet
        % and provided that the number of iterations allows for it
        while ~osc && (per <= cfg.maxper) && (per * (cfg.nper+1) + 1 < k+1)
            osc = 1;                % start by assuming oscillating; tests below might falsify that
            for i = 0:per-1,        % i loops through the offsets of the iters that have to be equal
                for j = 1:cfg.nper, % make sure oscillation happens for at least nper periods
                    osc = osc & all(abs(thetah{k+1-i} - thetah{k+1-j*per-i}) <= cfg.epsper);
                    if ~osc, break; end;    % don't test farther periods
                end;
                if ~osc, break; end;        % don't test other offets
            end;
            per = per + 1;
        end;
        per = per - 1;                      % revert the last increment
        if osc, kosc = k; conv = -1; end;   % flag oscillation
        timestat.iter_convtest(k) = cputime - tmark; clear tmark; % remainder not counted in exectime
        
        % update aggregate statistics
        timestat.iter_peval(k) = timestat.iter_computeAb(k) + timestat.iter_qsolve(k);
        timestat.iter_run(k) = timestat.iter_peval(k) + timestat.iter_convtest(k);
        timestat.run = timestat.run + timestat.iter_run(k);
        trun = timestat.run;                % keep trun for compatibility reasons
        
        % compute and store condition number for A
        if cfg.computecondA,
            if cfg.usesparse, condA(k) = condest(A); else condA(k) = cond(A); end;
        end;
        % store A and b if configured to do so
        if cfg.storeab, Ah{k} = A; bh{k} = b; end;
        
        % visual feedback of algorithm progress
        if ~mod(k, cfg.iterdisp),
            dispx(['k=' num2str(k) ' LSPI iter done, delta_k+1=' num2str(deltah(k+1))], cfg.verb, 2);
        end;
        % visualization
        if cfg.visualize && ~mod(k, cfg.itervis),
            vcfg.ell = k;
            [figh vcfg.gview] = feval(model.visualizefun, vcfg);
        end;
        % data backup
        if ~mod(k, cfg.itersave),
            save(cfg.datafile);
            dispx(['Data at iter k=' num2str(k) ' saved to [' cfg.datafile '].'], cfg.verb, 1);
        end;
        
        k = k + 1;
    end;        % WHILE not converged and allowed more iterations

    if conv,        dispx('Convergence detected. Algorithm stopped.', cfg.verb, 0);
    elseif osc,     dispx(['Solution oscillates with period ' num2str(per) '. Algorithm stopped'], cfg.verb, 0);
    elseif Anoninv, dispx('The A matrix is singular or badly conditioned. Algorithm stopped.', cfg.verb, 0);
    else            dispx(['maxiter=' num2str(cfg.maxiter) ' exhausted. Algorithm stopped'], cfg.verb, 0);
    end;
    
    % add last theta to the history
    thetah{k} = theta;
    
    % finalize visualizer
    if cfg.visualize,
         % make sure last iteration is visualized
        if vcfg.ell < k - 1,    vcfg.ell = k - 1;       % show last iteration
        else                    vcfg.lspiter = 0;       % don't show anything
        end;
        vcfg.finalize = 1;
        [figh vcfg.gview] = feval(model.visualizefun, vcfg);
    end;    
end;


% Options below might create figures, init figure handles array
figh = [];

% ==== REPLAY POLICY ====
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
            if ~isempty(mpfigh), figh(end+1) = mpfigh; end;
        else
            figh(end+1) = plothistory(hist);
        end;
%         % also put the return in the figure title
%         title(['R(x_0)=' num2str(hist.R(1))]);
        setfigprop(cfg);
        % save if requested
        if ~isempty(figh), saveplot(figh(end), [cfg.savedir cfg.savefig], cfg.plottarget); end;
    end;
end;        % IF replay

% ==== STATISTICS ====
if (cfg.evol || cfg.sol) && ~cfg.silent,
    % grayscale styles
    gs.cm = gray(96); gs.cm = gs.cm(24:end);
    % color styles
    cs = gs;
    cs.cm = jet(64);
    % set style
    if cfg.grayscale, sty = gs; else sty = cs; end;    
    
    % readable labels
    labels.iter = 'Iteration'; 
    labels.h = 'h(x)'; labels.V = 'V(x)'; 
    labels.delta = {'$\Vert \theta_\ell - \theta_{\ell-1}\Vert_2$', 'Interpreter', 'Latex', 'FontSize', 14}; 
    labels.condA = 'cond(A)';
end;

if cfg.sol && ~cfg.silent,
    if exist('Anoninv', 'var') && Anoninv,
        dispx('A was non-invertible, solution is meaningless. Skipping plot.', cfg.verb, 0);
    else
        figh(end+1) = figurex([1100 450]); % colormap(sty.cm);
        K = find(~isnan(deltah), 1, 'last') - 1;
        subplot(3, 2, [1 3]); cla;
        approx.plotv(approx, theta); title(labels.V);
        subplot(3, 2, [2 4 6]); cla;
        approx.ploth(approx, theta, 'npoints=100'); title(labels.h);
        subplot(3, 2, 5); cla;
        semilogy(1:K, deltah(2:K+1), 'k'); grid on; xlabel(labels.iter); ylabel(labels.delta{:});
        setfigprop(cfg);
        % save if requested
        saveplot(figh(end), [cfg.savedir cfg.savefig], cfg.plottarget);
    end;
end;
    
if cfg.evol && ~cfg.silent,
    if exist('cleanedup', 'var') && cleanedup >= 2,
        warning('Cannot replay evolution, hard cleanup was performed on the datafile.');
    elseif exist('Anoninv', 'var') && Anoninv,
        dispx('A was non-invertible. Skipping evolution plot.', cfg.verb, 0);
    else
        figh(end+1) = figure; colormap(sty.cm);
        K = find(~isnan(deltah), 1, 'last') - 1;
        for k = 1:K+1,
            figcfg.figname = sprintf('Iter#%d, %s', k, cfg.datafile);
            setfigprop(figcfg);
            subplot(221); cla;
            approx.plotv(approx, thetah{k}, 'npoints=30'); title(labels.V);
            subplot(222); cla;
            approx.ploth(approx, thetah{k}); title(labels.h);
            subplot(223); cla;
            semilogy(1:k-1, deltah(2:k), 'k'); xlabel(labels.iter); ylabel(labels.delta{:});
            if k <= K && exist('condA', 'var'),   % A not defined after the final iteration
                subplot(224); cla;
                semilogy(1:k, condA(1:k), 'k'); xlabel(labels.iter); ylabel(labels.condA);
            end;
            if k <= K, pause; end;
        end;
        % save if requested
        saveplot(figh(end), [cfg.savedir cfg.savefig], cfg.plottarget);
    end;
end;


% ==== BACKUP DATA ====
if cfg.run || cfg.resume, 
    % save into the same directory as the problem
    if isempty(cfg.datadir),
        datadir = fileparts(which(cfg.problem));
    else
        datadir = cfg.datadir; 
        if any(datadir(end) == '\/'), datadir = datadir(1:end-1); end;
    end;
    % if default file name is used, generate a unique file name
    if strcmp(cfg.datafile, CFG.datafile),
        c = fix(clock);
        for i = 1:length(c),
            cfg.datafile = [cfg.datafile '-' num2str(c(i), '%02d')];
        end;
        if exist([CFG.datafile '.mat'], 'file'), delete([CFG.datafile '.mat']); end;
    elseif ~strcmp(datadir, pwd)
        delete([cfg.datafile '.mat']);   % need to save in a different directory, so delete anyway
    end;    % otherwise just overwrite
    cfg.datafile = [datadir '/' cfg.datafile];
    save(cfg.datafile);
    dispx(['Data was saved to [' cfg.datafile '].'], cfg.verb, 1);
end;

% -----------------------------------------------
% set output
if cfg.run || cfg.resume,       % output optimal solution
    varargout = {theta, timestat};
elseif cfg.replay,              % output history and possibly figure handles
    varargout = {hist, figh};
elseif cfg.evol || cfg.sol,               % output fig handles (replay takes precedence)
    varargout = {figh};
end;

end
% lspi() RETURNING varargout =================================================================
