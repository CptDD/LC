function figh = dc_plot(cfg)
% Plot various information regarding the DC motor problem. Can also be used for inverted
% pendulums or in general 2-state, 1-action problems
%   FIGH = DC_PLOT(CFG)
% Parameters:
%   CFG         - history for a simple trajectory plot;
%               or full configuration of plot, see commented defaults for explanations
%
% Returns:
%   FIGH        - an (array of) handles to the created figure(s)

% default arguments
if nargin < 1, cfg = ''; end;
% support for default calling mode -- i.e. only history
if isstruct(cfg) && isfield(cfg, 't') && isfield(cfg, 'x'),
    hist = cfg;
    cfg = struct();
    cfg.traj = hist;
    cfg.datasource = 'caller';
end;

% where from to load the data, one of:
%       'caller'    - if the variables should be taken from the calling function
%       filename    - if data should be loaded from a file with name <filename>
CFG.problem = 'dc_problem';
CFG.datasource = 'dc_fzqi';
CFG.datafile   = '';    % data source can be given as a data file here as well
% What to plot
CFG.basis = 0;
% CFG.cefz = 1;         % whether a cross-entropy fuzzy algorithm was run
CFG.gridh = 0;          % plot grid h
CFG.gridq = 0;          % plot grid Q-function
CFG.fuzzyh = 0;         % if the fuzzy policy should be plotted
CFG.fuzzyv = 0;         % fuzzy value function
CFG.fuzzyq = 0;         % fuzzy Q-function
CFG.lspiq = 0;          % LSPI Q-function
CFG.lspih = 0;          % LSPI policy
CFG.fittedqiq = 0;      % fitted QI Q-function
CFG.fittedqih = 0;      % fitted QI policy
CFG.iter = [];          % plot Q-function and policy at WHICH iteration; only supported for LSPI
CFG.showlspiosamples = 0;    % show LSPI online samples

CFG.traj = [];          % or trajectory (history) structure

CFG.genh = 0;           % generic policy (specified in a function)
CFG.policy = '';        % generic policy to evaluate and arguments (static policy only)
CFG.policyargs = {};    
CFG.genq = 0;           % generic Q-function (specified in a function)
CFG.qfunction = '';     % generic Q-function function and arguments
CFG.qargs = {};

CFG.ucon = [];           % envelope constraints on u

CFG.rew = 0;
CFG.restrictx = [];     % restrict x domain in plots -- only supported in rew
CFG.rbfh = 0;           % RBF Q-iteration , resultingpolicy
CFG.rbfq = 0;           % RBF Q-iteration , resulting Q-value function
% CFG.rbfdirecth = 0;     % resulting policy from CE policy search with RBFs
CFG.interph = 0;        % if interpolated policy
                        % otherwise interpolate using linear basis function weights
CFG.Q = 0;              % if a constant-speed slice thru the Q-function should be plotted
% misc model params, when a model has to explicitly be created
CFG.model_params = {};
% plot configuration
CFG.slice = [NaN NaN 0];
CFG.gridres = 100;
CFG.enhance = []; CFG.enhfactor = 2;
CFG.markersize = 4;
CFG.posstep = .05;      % the pgrid step for plotting policies
CFG.figxopt = [];
CFG = setfigprop(CFG, 'addfields');  % add figure settings fields to CFG
% save configuration
CFG.plottarget = 'screen';     % 'latex', 'beamer', 'screen', ''
% only last created figure can be saved using these options
CFG.savedir = ''; % 'D:\Work\tex\papers\alamas07\img\';
                        % path for saving figure
CFG.savefig = 'dc_evol';    % filename for saving figure

% process config
cfg = parseconfig(cfg, CFG);
if ~isempty(cfg.datafile), cfg.datasource = cfg.datafile; end;
% cfg           % feedback on config

% needed vars
% XMFS only required for fuzzy fuzzyh plot
vars = {'model', 'cfg'};
if cfg.fuzzyq || cfg.fuzzyh || cfg.fuzzyv, 
    vars = {vars{:}, 'X', 'U', 'DIMS', 'theta', 'XMFS'};
end;
if cfg.rbfq || cfg.rbfh,
    vars = {vars{:}, 'c', 'rad', 'U', 'DIMS', 'theta'};
end;
if cfg.gridh || cfg.gridq,
    vars = {vars{:}, 'X', 'U', 'X0', 'DIMS', 'theta'};
end;
if cfg.lspiq || cfg.lspih,
    vars = {vars{:}, 'approx', 'theta', 'htheta', 'happrox'};
    if ~isempty(cfg.iter),  % other iterations may be needed
        vars  = {vars{:}, 'thetah', 'hthetah'};
    end;
end;
if cfg.fittedqiq || cfg.fittedqih,
    vars = {vars{:}, 'approx'};
end;

% load model, grids, etc. from the data source -- for the options that
% require it
if cfg.fuzzyq || cfg.fuzzyh || cfg.fuzzyv || cfg.gridh || cfg.gridq || ...
        cfg.genh || cfg.genq || cfg.rbfq || cfg.rbfh || cfg.lspiq || cfg.lspih || cfg.fittedqiq || cfg.fittedqih, 
    cfg1 = cfg;
    switch cfg.datasource,
        case 'caller',              % load needed vars from caller space
            for i=1:length(vars),
                cv.(vars{i}) = evalin('caller', vars{i});
            end;
            structtovars(cv);
        otherwise                   % load from file
            % some variables may not be found and it's OK (namely, policy-related stuff when
            % actually loading from an LSPI, not LSPIH, result)
            warning('off', 'MATLAB:load:variableNotFound');        
            load(cfg.datasource, vars{:});
            warning('on', 'MATLAB:load:variableNotFound');        
    end;
    expcfg = cfg;   % the config from the file
    cfg = cfg1;     % the saved config
    clear cfg1;     % intermediary variable
elseif ~isempty(cfg.rew) || ~isempty(cfg.ucon),
    % create a model
    if isscalar(cfg.rew) && cfg.rew == 1,   % cfg.rew just a flag, config in model_params
        model = feval(cfg.problem, 'model', cfg.model_params{:});
    elseif ischar(cfg.rew), % cfg.rew is the reward config
        model = feval(cfg.problem, 'model', cfg.rew, cfg.model_params{:});
    else
        model = feval(cfg.problem, 'model', cfg.model_params{:});        
    end;
end;

p = model.p; q = model.q;

% grayscale styles
gs.schemec = 'k'; 
gs.innerc = [1 1 1]; gs.centerc = .25 * [1 1 1];
gs.cm = gray(128); gs.cm = gs.cm(33:end, :);    % w/o strong blacks and whites
gs.mesh  = {'EdgeColor', [.3 .3 .3]};   % use dark meshes for readability
% color styles
cs.schemec = 'k'; cs.innerc = 'b'; cs.centerc = 'r'; cs.cm = jet;
cs.mesh = {}; %{'EdgeColor', [.3 .3 .3]};
% set style
if cfg.grayscale, sty = gs; 
else sty = cs; end;

% readable labels
commonprop = {'Interpreter', 'LaTeX', 'FontSize',13};
rl.xu = {'\alpha', '\alpha''', 'u'}; rl.xuunits = {'[rad]', '[rad/s]', '[V]'};
rl.x = {'\alpha [rad]'}; rl.y = {'\alpha'' [rad/s]'};  
rl.h = {'h(\alpha,\alpha'') [V]'}; rl.q = 'Q'; rl.v = {'V(\alpha,\alpha'')'};
% x1, x2 version
% rl.xu = {'x_1', 'x_2', 'u'}; 
% rl.x = {'x_1'}; rl.y = {'x_2'};  
% rl.h = {'h(x_1,x_2)'}; rl.q = 'Q'; rl.v = {'V(x_1,x_2)'};
% Old, latex versions
% rl.x = {'$x_1$',commonprop{:}}; rl.y = {'$x_2$',commonprop{:}}; 
% rl.h = {'$h(x_1,x_2)$',commonprop{:}}; 
% rl.q = 'Q';
% rl.v = {'$V^*(x_1,x_2)$', commonprop{:}};
rl.Rlqr = '\rho'; rl.RBigBox = ' \rho'; rl.Rbox = '\rho';
rl.Rshaping = '\rho'''; rl.Rshapbox = '\rho''';
labels = rl;

if cfg.fuzzyq || cfg.fuzzyv || cfg.fuzzyh,
    % "translate" the variables in the CEfuzzy format for uniformity of code below
    N = DIMS.N;
    M = DIMS.M;
    Np = DIMS.dimx;
    cgridstar = X;
    thetastar = theta;
    U = U{1};       % equivalent to U = flat(U)
    expcfg.roll = 0*Np;
end;

if cfg.lspiq || cfg.lspih, % || cfg.fittedqih || cfg.fittedqih -- to save computation, commented out 
    % revise the approximator
    approx = revise_approx(approx);
end;

% compute grids if only state space is interesting
% if cfg.rbfdirecth || cfg.rbfh || cfg.rbfv || cfg.fuzzyh,
if cfg.fuzzyv || cfg.fuzzyh || cfg.gridh || cfg.genh || cfg.rbfh || cfg.lspih ||~isempty(cfg.ucon) || cfg.fittedqih,
    if ~isempty(cfg.restrictx), maxx = cfg.restrictx;
    else maxx = model.maxx;
    end;
    pgrid.vertices = {-maxx(1):2*maxx(1)/cfg.gridres:maxx(1), ...
        -maxx(2):2*maxx(2)/cfg.gridres:maxx(2)};
    pgrid.points = flat(pgrid.vertices);
    pgrid.size = [length(pgrid.vertices{1}) length(pgrid.vertices{2})];
    pgrid.npoints = size(pgrid.points, 2);
% select slice and compute grids, for options that require 3-D space
elseif cfg.rew || cfg.genq || cfg.fuzzyq || cfg.rbfq || cfg.lspiq || cfg.gridq || cfg.fittedqiq,
    % which vars to expand
    expandvars = find(isnan(cfg.slice));
    if ~isempty(cfg.restrictx), maxx = cfg.restrictx;
    else maxx = model.maxx;
    end;
    maxv = [maxx(:); model.maxu];
    pgrid.vertices = cell(p+q, 1);
    pgrid.arglist = '(';
    pgrid.size = [];
    for i = 1:p+q,
        if any(i == expandvars),
            if i > p && (cfg.fuzzyq || cfg.rbfq),   % TODO should also check for discrete-action approx!
                % vertices can only be discrete actions
                vertices = U;       % just 1 action dimension for this problem
            else    % vertices are free
                vertices = -maxv(i):2*maxv(i)/cfg.gridres:maxv(i);
                % enhance resolution in desired areas, if specified
                if ~isempty(cfg.enhance) && ~isempty(cfg.enhance{i}),
                    h = 2*maxv(i)/cfg.gridres;  % large step
                    for ii = 1:length(cfg.enhance{i}),
                        vertices = [vertices (cfg.enhance{i}(ii) + (-h:h/cfg.enhfactor:h))];
                    end;
                    vertices = sortx(vertices);
                end;
            end;
            pgrid.vertices{i} = vertices;
            pgrid.size(end+1) = length(pgrid.vertices{i});
            pgrid.arglist = [pgrid.arglist rl.xu{i} ','];
        else
            pgrid.vertices{i} = cfg.slice(i);
            pgrid.arglist = [pgrid.arglist num2str(cfg.slice(i)) ','];
        end;
    end;
    pgrid.arglist(end) = ')'; % overwriting comma
    
    pgrid.points = flat(pgrid.vertices);
    pgrid.npoints = size(pgrid.points, 2);
end;


% ------------------------
% PREPARE BF VALUES TO AVOID CODE DUPLICATION

% for fuzzy options
if cfg.fuzzyh || cfg.fuzzyq || cfg.fuzzyv,
    tab = dec2base(0:2^model.p-1, 2) - 47;       % auxiliary indices table
    MU = cell(pgrid.npoints, 1);
    % No rollover supported!
    for i = 1:pgrid.npoints,
        [ind, mu] = mdegs_p(pgrid.points(1:p, i), cgridstar, expcfg.roll, Np, model.p, tab);
        MU{i} = [ind mu];
    end;
end;
% RBF options
if cfg.rbfq || cfg.rbfh,
    RBFS = zeros(pgrid.npoints, DIMS.N);
    for i = 1:pgrid.npoints,
        RBFS(i, :) = nrbf(pgrid.points(1:p, i), DIMS.N, c, rad);        
    end;
end;

% ------------------------------
% PLOT DATA

figh = [];

% -----------
% Reward function
if cfg.rew,
    R = zeros(1, pgrid.npoints);
    for i = 1:pgrid.npoints,
        [xp R(i)] = model.fun(model, pgrid.points(1:p, i), pgrid.points(p+1, i));
    end;
    clear xp;
    
    figh(end+1) = figurex; 
    mesh(pgrid.vertices{expandvars(1)}, pgrid.vertices{expandvars(2)}, reshape(R, pgrid.size)', sty.mesh{:});
    colormap(sty.cm);

    if isempty(cfg.xlim),
        cfg.xlim = [-maxv(expandvars(1)), maxv(expandvars(1))];
    end;
    if isempty(cfg.ylim),
      	cfg.ylim = [-maxv(expandvars(2)), maxv(expandvars(2))];
    end;
    setfigprop(cfg);

    xlabel([labels.xu{expandvars(1)} ' ' labels.xuunits{expandvars(1)}]); 
    ylabel([labels.xu{expandvars(2)} ' ' labels.xuunits{expandvars(2)}]); 
    if isfield(labels, ['R' model.rewtype]),
        zlabel([labels.(['R' model.rewtype]) pgrid.arglist]);
    end;
end;

% ------ Generic policy
if cfg.genh,
    h = zeros(model.q, pgrid.npoints);
     for i = 1:pgrid.npoints,
         h(:, i) = feval(cfg.policy, pgrid.points(:, i), [], cfg.policyargs{:});
         % [] stands for (unused) time argument
    end;

    figh(end+1) = figure;
    ph = pcolor(pgrid.vertices{1}, pgrid.vertices{2}, reshape(h(1, :), pgrid.size)');
    set(ph, 'LineStyle', 'none'); hold on; 
    xlabel(labels.x{:}); ylabel(labels.y{:}); title(labels.h{:});
    box on; colormap(gs.cm);
    setfigprop(cfg);

end;

% ------ RBF policy
if cfg.rbfh,
    
    h = zeros(model.q, pgrid.npoints);
    if cfg.interph,
    else
         for i = 1:pgrid.npoints,
            Qa = RBFS(i, :) * theta;
            [Qstar ui] = max(Qa);
            h(:, i) = U(:, ui);
        end;
    end;

    figh(end+1) = figure; 
    ph = pcolor(pgrid.vertices{1}, pgrid.vertices{2}, reshape(h(1, :), pgrid.size)');
    set(ph, 'LineStyle', 'none'); hold on; 
    xlabel(labels.x{:}); ylabel(labels.y{:}); title(labels.h{:});
    box on; colormap(gs.cm); 
    setfigprop(cfg);
end;

% ------ Fuzzy policy
if cfg.fuzzyh,
    
    h = zeros(model.q, pgrid.npoints);
    if cfg.interph,
        [Qstar ui] = max(thetastar, [], 2);
        hstar = U(:, ui)';
        for i = 1:pgrid.npoints,
            h(:, i) = MU{i}(:, 2)' * hstar(MU{i}(:, 1), :);
        end;
    else
        ties = 0;
        for i = 1:pgrid.npoints,
            Qa = MU{i}(:, 2)' * thetastar(MU{i}(:, 1), :);
            [Qstar ui] = max(Qa);
%             if sum((Qstar - Qa) <= 1e-5) > 1, 
%                 ties = ties + 1; 
%                 xtie = pgrid.points(1:p, i)
%             end;
            h(:, i) = U(:, ui);
% this looks like a left-over bug from the car on the hill?
%             if Qa(1) ~= Qa(2),
%                 [Qstar ui] = max(Qa);
%                 h(:, i) = U(:, ui);
%             else h(:, i) = 0;       % i.e., gray
%             end;
        end;
%         ties
    end;

    figh(end+1) = figure; 
    HM = reshape(h(1, :), pgrid.size);
    ph = pcolor(pgrid.vertices{1}, pgrid.vertices{2}, HM');
    set(ph, 'LineStyle', 'none'); hold on; 
    xlabel(labels.x{:}); ylabel(labels.y{:}); title(labels.h{:});
    box on; colormap(gs.cm); setfigprop(cfg);
    
    if cfg.basis, plotfuzzybasis(cgridstar, sty, cfg); end;

end;

% ------ Grid policy
if cfg.gridh,
    h = zeros(model.q, pgrid.npoints);
    [Qstar ui] = max(theta, [], 2);
    for i = 1:pgrid.npoints,
        h(:, i) = ui(ndi2lin( findbox(X, pgrid.points(:, i)), DIMS.dimx ));
    end;
    figh(end+1) = figure; 
    % translate indices into actual actions
    % make sure only 1 action variable is retained 
    HM = reshape(U{1}(h(1, :)), pgrid.size);   
    ph = pcolor(pgrid.vertices{1}, pgrid.vertices{2}, HM');
    set(ph, 'LineStyle', 'none'); hold on; box on;
    xlabel(labels.x{:}); ylabel(labels.y{:}); title(labels.h{:});
    box on; colormap(gs.cm); setfigprop(cfg);
% % old version, plots using samples X0    
%     U = U{1};
%     h = zeros(DIMS.N, 1);
%     for i = 1:DIMS.N,
%         Qa = theta(i, :);
%         h(i) = U(Qa == max(Qa));
%     end;
%     figh(end+1) = figure; 
%     ph = pcolor(X0{1}, X0{2}, reshape(h, DIMS.dimx)');
%     set(ph, 'LineStyle', 'none'); hold on; 
%     xlabel(labels.x{:}); ylabel(labels.y{:}); title(labels.h{:});
%     box on; colormap(gs.cm); 
%     setfigprop(cfg);    
end;

% ------ Grid Q function
if cfg.gridq,
    Q = zeros(1, pgrid.npoints);
    for i = 1:pgrid.npoints,
        Q(i) = theta(ndi2lin(findbox(X, pgrid.points(1:p, i)), DIMS.dimx), ...
            findflat(pgrid.points(p+1, i), U{1}));
    end;
    figh(end+1) = figure;
    mesh(pgrid.vertices{expandvars(1)}, pgrid.vertices{expandvars(2)}, reshape(Q, pgrid.size)', sty.mesh{:});
    xlabel([labels.xu{expandvars(1)} ' ' labels.xuunits{expandvars(1)}]); 
    ylabel([labels.xu{expandvars(2)} ' ' labels.xuunits{expandvars(2)}]); 
    zlabel([labels.q pgrid.arglist]);
    xlim([-model.maxx(1) model.maxx(1)]); ylim([-model.maxx(2) model.maxx(2)]);
    colormap(sty.cm); setfigprop(cfg);
end;

% ------ Fuzzy value function
if cfg.fuzzyv,
    V = zeros(1, pgrid.npoints);
    for i = 1:pgrid.npoints,
        V(i) = max(MU{i}(:, 2)' * thetastar(MU{i}(:, 1), :));
    end;

    figh(end+1) = figure;
    mesh(pgrid.vertices{1}, pgrid.vertices{2}, reshape(V, pgrid.size)', sty.mesh{:});
    
    xlabel(labels.x{:}); ylabel(labels.y{:});
    zlabel(labels.v{:});
    colormap(sty.cm); setfigprop(cfg);    

    % old way of plotting the V function
%     surf(cgridstar{1}, cgridstar{2}, thetastar', 'EdgeColor', [1 1 1]*.5); hold on;
%     if cfg.basis,
%         surface(cgridstar{1}, cgridstar{2}, thetastar', 'EdgeColor', 'none', 'FaceColor', 'none', ...
%             'Marker' ,'o', 'MarkerFaceColor', 'r', 'MarkerSize', cfg.markersize);
%     end;

end;

% ------ Fuzzy Q function
if cfg.fuzzyq,
    Q = zeros(1, pgrid.npoints);
    for i = 1:pgrid.npoints,
        Q(i) = MU{i}(:, 2)' * thetastar(MU{i}(:, 1), findflat(pgrid.points(p+1, i), U));
    end;

    figh(end+1) = figure;
    mesh(pgrid.vertices{expandvars(1)}, pgrid.vertices{expandvars(2)}, reshape(Q, pgrid.size)', sty.mesh{:});
    xlabel([labels.xu{expandvars(1)} ' ' labels.xuunits{expandvars(1)}]); 
    ylabel([labels.xu{expandvars(2)} ' ' labels.xuunits{expandvars(2)}]); 
    zlabel([labels.q pgrid.arglist]);
    xlim([-model.maxx(1) model.maxx(1)]); ylim([-model.maxx(2) model.maxx(2)]);
    colormap(sty.cm); setfigprop(cfg);

end;

% ------ Generic Q function
if cfg.genq,
    Q = zeros(1, pgrid.npoints);
    for i = 1:pgrid.npoints,
         Q(i) = feval(cfg.qfunction, pgrid.points(1:p, i), pgrid.points(p+1:end, i), cfg.qargs{:});
    end;

    figh(end+1) = figure;
    mesh(pgrid.vertices{expandvars(1)}, pgrid.vertices{expandvars(2)}, reshape(Q, pgrid.size)', sty.mesh{:});
    xlabel([labels.xu{expandvars(1)} ' ' labels.xuunits{expandvars(1)}]); 
    ylabel([labels.xu{expandvars(2)} ' ' labels.xuunits{expandvars(2)}]); 
    zlabel([labels.q pgrid.arglist]);
    xlim([-model.maxx(1) model.maxx(1)]); ylim([-model.maxx(2) model.maxx(2)]);
    colormap(gs.cm); setfigprop(cfg);

end;

% RBF Q-function
if cfg.rbfq,
    figh(end+1) = figure; clf;
    
    Q = zeros(1, pgrid.npoints);
    for i = 1:pgrid.npoints,
        Q(i) = RBFS(i, :) * theta(:, findflat(pgrid.points(p+1, i), U));
    end;

    figh(end+1) = figure;
    mesh(pgrid.vertices{expandvars(1)}, pgrid.vertices{expandvars(2)}, reshape(Q, pgrid.size)', sty.mesh{:});
    xlabel([labels.xu{expandvars(1)} ' ' labels.xuunits{expandvars(1)}]); 
    ylabel([labels.xu{expandvars(2)} ' ' labels.xuunits{expandvars(2)}]); 
    zlabel([labels.q pgrid.arglist]);
    xlim([-model.maxx(1) model.maxx(1)]); ylim([-model.maxx(2) model.maxx(2)]);
    setfigprop(cfg);
    colormap(gs.cm);
end;

% Q-function computed with LSPI algorithms
if cfg.lspiq,   
    % this should be perhaps updated to use approx_plot*
    if ~isempty(cfg.iter),  % select given iteration
        theta = thetah{cfg.iter+1};
    end;
    Q = zeros(1, pgrid.npoints);
    for i = 1:pgrid.npoints,
        Q(i) = approx.q(approx, theta, pgrid.points(1:p, i), pgrid.points(p+1:end, i));
    end;
    figh(end+1) = figure;
    mesh(pgrid.vertices{expandvars(1)}, pgrid.vertices{expandvars(2)}, reshape(Q, pgrid.size)', sty.mesh{:});
    xlabel([labels.xu{expandvars(1)} ' ' labels.xuunits{expandvars(1)}]); 
    ylabel([labels.xu{expandvars(2)} ' ' labels.xuunits{expandvars(2)}]); 
    zlabel([labels.q pgrid.arglist]);
    xlim([-model.maxx(1) model.maxx(1)]); ylim([-model.maxx(2) model.maxx(2)]);
    colormap(gs.cm); setfigprop(cfg);
end;

% policy computed with LSPI algorithms
if cfg.lspih,
    % this should be perhaps updated to use approx_plot*
    if ~isempty(cfg.iter),  % select given iteration
        if isfield(expcfg, 'happrox'),    % lspih was used
            htheta = hthetah{cfg.iter};
        else
            theta = thetah{cfg.iter};
        end;
    end;
    h = zeros(1, pgrid.npoints);
    for i = 1:pgrid.npoints,
        if isfield(expcfg, 'happrox'),    % lspih was used
            h(i) = happrox.h(happrox, htheta, pgrid.points(1:p, i));
        else 
            h(i) = approx.h(approx, theta, pgrid.points(1:p, i));
        end;
    end;
    figh(end+1) = figure;
    ph = pcolor(pgrid.vertices{1}, pgrid.vertices{2}, reshape(h(1, :), pgrid.size)');
    set(ph, 'LineStyle', 'none'); hold on; 
    xlabel(labels.x{:}); ylabel(labels.y{:}); title(labels.h{:});
    if cfg.showlspiosamples, % show samples for online methods
        load(cfg.datasource, 'X');
        % subsample to avoid a huge figure
        x1 = reshape(X(1, :, 1:5:end), 1, []); 
        x2 = reshape(X(2, :, 1:5:end), 1, []);
        valid = ~isnan(x1) & ~isnan(x2);
        x1 = x1(valid); x2 = x2(valid);
        plot(x1, x2, 'LineStyle', 'none', 'MarkerEdgeColor', 'none', 'MarkerFaceColor', [.4 .4 .4], ...
            'Marker', 'o', 'MarkerSize', 2);
    end;
    box on; colormap(gs.cm); setfigprop(cfg);
end;


% Q-function computed with fitted QI
if cfg.fittedqiq,   
    % this should be perhaps updated to use approx_plot*
    if approx.vectorized,   % use vectorized call for efficiency
        Q = approx.q(approx, [], pgrid.points(1:p, :), pgrid.points(p+1:end, :));
    else
        Q = zeros(1, pgrid.npoints);
        for i = 1:pgrid.npoints,
            Q(i) = approx.q(approx, [], pgrid.points(1:p, i), pgrid.points(p+1:end, i));
        end;
    end;
    figh(end+1) = figure;
    mesh(pgrid.vertices{expandvars(1)}, pgrid.vertices{expandvars(2)}, reshape(Q, pgrid.size)', sty.mesh{:});
    xlabel([labels.xu{expandvars(1)} ' ' labels.xuunits{expandvars(1)}]); 
    ylabel([labels.xu{expandvars(2)} ' ' labels.xuunits{expandvars(2)}]); 
    zlabel([labels.q pgrid.arglist]);
    xlim([-model.maxx(1) model.maxx(1)]); ylim([-model.maxx(2) model.maxx(2)]);
    colormap(gs.cm); setfigprop(cfg);
end;

% policy computed with fitted QI
if cfg.fittedqih,   
    % this should be perhaps updated to use approx_plot*
    if approx.vectorized,   % use vectorized call for efficiency
        h = approx.h(approx, [], pgrid.points);
    else
        h = zeros(1, pgrid.npoints(1:p, :));
        for i = 1:pgrid.npoints,
            h(i) = approx.h(approx, [], pgrid.points(1:p, i));
        end;
    end;
    figh(end+1) = figure;
    ph = pcolor(pgrid.vertices{1}, pgrid.vertices{2}, reshape(h, pgrid.size)');
    set(ph, 'LineStyle', 'none'); hold on; 
    xlabel(labels.x{:}); ylabel(labels.y{:}); title(labels.h{:});
    box on; colormap(gs.cm); setfigprop(cfg);
end;

if ~isempty(cfg.traj),
%     commonprop = {'Interpreter', 'LaTeX', 'FontSize',14};
    if cfg.addtocrt == 1,
        styles = {{'k-','LineWidth',2,'Color',[.6,.6,.6]}, {'k-','LineWidth',2,'Color',[.6,.6,.6]}}; 
    elseif cfg.addtocrt == 2,
        styles = {{'k-','LineWidth',1}, {'k-','LineWidth',1}}; 
    else    % regular style
        styles = {{'k-','LineWidth',1}, {'k-','LineWidth',2,'Color',[.6,.6,.6]}}; 
    end;
    h = cfg.traj;
    if isempty(cfg.addtocrt) || ~cfg.addtocrt, figh(end+1) = figure;
    else figh(end+1) = gcf;
    end;
    subplot(7, 1, [1 2]); if cfg.addtocrt, hold on; end;
    plot(h.t, h.x(1, :), styles{1}{:});
    ylabel('\alpha [rad]'); % ylabel('x_1 [rad]');
    grid on; box off;
    subplot(7, 1, [3 4]); if cfg.addtocrt, hold on; end;
    plot(h.t, h.x(2, :), styles{1}{:});
    ylabel('\alpha'' [rad/s]'); % ylabel('x_2 [rad/s]'); 
    grid on;  box off;
    subplot(7, 1, [5 6]); if cfg.addtocrt, hold on; end;
    stairs(h.t, h.u, styles{1}{:});
    ylabel('u [V]'); grid on;  box off;
    subplot(7, 1, 7); if cfg.addtocrt, hold on; end;
    plot(h.t, h.r, styles{2}{:});grid on;  box off;
    ylabel('r [-]'); xlabel('t [s]');
    colormap(gs.cm); setfigprop(cfg);
end;

% envelope constraints on the action
if cfg.ucon,
    UCON = zeros(2, pgrid.npoints);
    uconargs = feval(cfg.ucon, 'init', model);
    for i = 1:pgrid.npoints,
        [UCON(1, i), UCON(2, i)] = feval(cfg.ucon, pgrid.points(1:p, i), uconargs{:});
    end;
    figh(end+1) = figure;
    mesh(pgrid.vertices{1}, pgrid.vertices{2}, reshape(UCON(1, :), pgrid.size)', 'EdgeColor', 'b');
    hold on;
    mesh(pgrid.vertices{1}, pgrid.vertices{2}, reshape(UCON(2, :), pgrid.size)', 'EdgeColor', 'r');
    legend('Lower bound', 'Upper bound');
    colormap(gs.cm); setfigprop(cfg);
end;

% save last figure if indicated
if ~isempty(figh),
    if ~isempty(cfg.datafile),
        set(figh(end), 'Name', cfg.datafile, 'NumberTitle', 'off');
    end;
    saveplot(figh(end), [cfg.savedir cfg.savefig], cfg.plottarget);
end;

end
% END nav_plot RETURNING array of figure handles =================================================


function plotfuzzybasis(cgridstar, sty, cfg)
[pts1 pts2] = ndgrid(cgridstar{1}, cgridstar{2});
plot(pts1(:), pts2(:), 'LineStyle', 'none', 'Color', sty.centerc, ...
            'Marker', 'o', 'MarkerSize', cfg.markersize, 'MarkerFaceColor', sty.innerc);
end