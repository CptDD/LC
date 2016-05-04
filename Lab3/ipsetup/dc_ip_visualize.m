function varargout = dc_ip_visualize(cfg)
% Visualization function for DC motor and inverted pendulum

% REMARK 'step' (for online algs) visualization only supported independently from 'iter'
% TODO visualizing from data file: not tested
% output to sequence of figures: not tested

% default arguments
if nargin < 1, cfg = ''; end;

% where from to load the data, one of:
%       'caller'    - if the variables should be taken from the calling function
%       filename    - if data should be loaded from a file with name <filename>
% CFG.problem = 'cleanrob_problem';
CFG.datasource = 'caller';
CFG.gview = [];           % if nonempty: update existing; if empty: create new; graphical view
% CFG.datafile   = '';    % data source can be given as a data file here as well
% update/create view with:
CFG.fuzzyqiter = 0;           
CFG.llrfittedqiter = 0;
CFG.lspiter = 0;
CFG.lspionlinestep = 0;
CFG.lspihonlinestep = 0;
CFG.approxqlearnstep = 0;
CFG.approxqlearntrial = 0;
CFG.mpcstep = 0;
CFG.finalize = 0;         % special option to finalize the output (namely, movie)
% which qiter/piter iteration/trial to plot
CFG.ell = [];
% CFG.tau = [];
CFG.trial = [];
CFG.k = [];
CFG.ktotal = [];        % total #of steps from start of experiment
% % NOT YET SUPPORTED target Q-function/policy for performance plots
% CFG.Qstar = [];
% CFG.hstar = [];
% CFG.Qhstar = [];

CFG = setfigprop(CFG, 'addfields');  % add figure settings fields to CFG
% output configuration
CFG.slice = [NaN NaN 0];
CFG.gridres = 50;
CFG.pause = .1;
CFG.plottarget = 'screen';      % 'latex', 'beamer', 'screen', 'movie'
CFG.snapshottarget = 'screen';  % 'latex', 'beamer', 'screen'
CFG.savedir = ''; 
CFG.savefig = 'dcvisualization';% filename for saving figure
% if output is "movie":
CFG.compression = 'Cinepak';    % quality with Cinepak is pretty crappy...
CFG.quality = 100;
CFG.fps = 1;
CFG.snapshot = [];              % still snapshot config
% snapshot has the form [iter]

% process config
cfg = parseconfig(cfg, CFG);

% needed vars
vars = {'model', 'cfg'};
if cfg.fuzzyqiter,
    vars = {vars{:}, 'X', 'U', 'DIMS', 'qistats'};
elseif cfg.llrfittedqiter,
    vars = {vars{:}, 'Xs', 'Is', 'Qsh'};
elseif cfg.lspiter,
    vars = {vars{:}, 'approx', 'thetah', 'deltah'};
elseif cfg.lspionlinestep || cfg.lspihonlinestep || cfg.approxqlearnstep || cfg.mpcstep,
    vars = {vars{:}, 'X'}; 
elseif cfg.approxqlearntrial,
    vars = {vars{:}, 'X', 'R'}; 
end;

% retrieve needed variables from the data source
cfg1 = cfg;
switch cfg.datasource,
    case 'caller',              % load needed vars from caller space
        for i=1:length(vars), 
            eval(sprintf('%s=evalin(''caller'', ''%s'');', vars{i}, vars{i})); 
        end;
    otherwise,
%         load(cfg.datasource, vars{:});
end;
expcfg = cfg; cfg = cfg1;

% revise any approximator used
if cfg.lspiter, approx = revise_approx(approx); end;

% set up style...
sty = struct;
sty.cm = jet;
% sty.hcm = gray(128); sty.hcm = sty.hcm(33:end, :);    % w/o strong blacks and whites
sty.title = {'HorizontalAlignment', 'left', 'FontSize', 12, 'FontWeight', 'bold', 'Margin', 1, 'LineStyle', 'none'};
sty.delta = {'Color', 'b', 'LineWidth', 1.5};
sty.bulletsize = .1;
sty.bulletcolor = 'k';
sty.tickspacing = 30;  % in degrees
sty.tickcolor = 'k';
sty.maintickcolor = 'r';
sty.mainticksize = .1;
sty.return = {'Color', 'g', 'LineWidth', 1.5};

% ...and labels
labels = struct;
labels.x = {'\alpha', '\alpha'''}; labels.u = {'u'};  
labels.h = {'h(\alpha,\alpha'')'}; labels.Q = 'Q(\\alpha,\\alpha'',%g)';
labels.iter = 'ell';
labels.deltatheta = '\theta_{ell+1} - \theta_{ell}';
labels.trial = 'trial'; labels.return = 'R(x_0)';

% ===================================
% ==== Prepare data for plotting ====

% flags for what we are going to show
show = struct;
show.world = [];
show.x = [];
show.Q = [];
show.h = [];
show.delta = [];
show.return = [];
show.nothing = 0;

% prepare grid
if cfg.fuzzyqiter || cfg.llrfittedqiter || cfg.lspiter,
    pgrid.vertices = {-model.maxx(1):2*model.maxx(1)/cfg.gridres:model.maxx(1), ...
        -model.maxx(2):2*model.maxx(2)/cfg.gridres:model.maxx(2)};
    pgrid.points = flat(pgrid.vertices);
    pgrid.size = [length(pgrid.vertices{1}) length(pgrid.vertices{2})];
    pgrid.npoints = size(pgrid.points, 2);    
    if ~isnan(cfg.slice(1)) || ~isnan(cfg.slice(2)),
        error('DC_IP_VISUALIZE: Only slices of the form [NaN,NaN,u] are supported');
    end;
    pgrid.u = cfg.slice(3);
    % insert actual action into the Q label
    labels.Q = sprintf(labels.Q, pgrid.u);
end;    

% compute quantities to show
if cfg.fuzzyqiter,
    % pick up theta after iteration ell
    theta = qistats.theta{cfg.ell+1};
    % compute Q slice and h
    show.Q = zeros(pgrid.size);
    show.h = zeros(pgrid.size);
    usliceind = find(pgrid.u == U{1}, 1, 'first');    % exploit fact that action is 1-D
    p = model.p;
    tab = dec2base(0:2^p-1, 2) - 47;       % auxiliary indices table
    roll = 0*DIMS.dimx; % No rollover supported!
    for i = 1:pgrid.npoints,
        % find nonzero MFs and their indices
        [muind, mu] = mdegs_p(pgrid.points(1:p, i), X, roll, DIMS.dimx, model.p, tab);
        % compute Q-values of all the actions in the current state
        Qx = mu' * theta(muind, :);
        % find the Q-value of the discrete action for the Q-slice
        show.Q(i) = Qx(usliceind);
        % find the greedy action and add to policy
        [Qmax umaxind] = max(Qx);
        show.h(i) = U{1}(umaxind);   % exploit action 1-D
    end;
    % also show delta
    show.deltaxaxis = 1:cfg.ell;
    show.delta = qistats.delta(1:cfg.ell);
    show.Qsubplot = {3, 2, [1 3]};
    show.hsubplot = {3, 2, [2 4]};
    show.deltasubplot = {3, 2, [5 6]};
     % customize labels
    labels.xdelta = labels.iter;        
    labels.deltalegend = labels.deltatheta;      
    gtitle = sprintf('Fuzzy Q-iteration, ell=%d', cfg.ell);
% compute quantities to show
elseif cfg.llrfittedqiter,
    % create an appropriate approximator (to allow generic Q and h computation)
    acfg = struct;
    acfg.U = expcfg.U;
    acfg.knnmethod = expcfg.knnmethod; acfg.knn = expcfg.knn; 
    acfg.S = expcfg.S;
    acfg.wfun = expcfg.wfun;
    acfg.linsolvemethod = expcfg.linsolvemethod;
    acfg.extrap = expcfg.extrap;
    acfg.Xs = Xs; acfg.Is = Is; acfg.Qs = Qsh{cfg.ell+1}; % note we pick the appropriate iterate
    if strcmp(expcfg.knnmethod, 'kdtree'), 
        error('dc_ip_visualize does not yet support kdtrees (2-stage loading needed...)');
        % cfg.kdtrees = kdtrees; 
    end;
    approx = llrdisc(model, acfg); clear acfg;
    % compute Q slice and h
    show.Q = zeros(pgrid.size);
    show.h = zeros(pgrid.size);
    for i = 1:pgrid.npoints,
        show.Q(i) = approx.q(approx, [], pgrid.points(1:model.p, i), pgrid.u);
        show.h(i) = approx.h(approx, [], pgrid.points(1:model.p, i));
    end;
    show.Qsubplot = {1, 2, 1};
    show.hsubplot = {1, 2, 2};
    % customize labels
    gtitle = sprintf('LLR Fitted Q-iteration, ell=%d', cfg.ell);
elseif cfg.lspiter,
    % pick up theta after iteration ell
    theta = thetah{cfg.ell+1};
    % compute Q slice and h
    show.Q = zeros(pgrid.size);
    show.h = zeros(pgrid.size);
    p = model.p;
    for i = 1:pgrid.npoints,
        show.Q(i) = approx.q(approx, theta, pgrid.points(1:p, i), pgrid.u);
        show.h(i) = approx.h(approx, theta, pgrid.points(1:p, i));
    end;
    % also show delta
    show.deltaxaxis = 1:cfg.ell;
    show.delta = deltah(2:cfg.ell+1); % note offset of 1, to correspond to thetah indices
    % show Q before h (like for Q-iteration) since this h is in fact
    % computed from Q
    show.Qsubplot = {3, 2, [1 3]};
    show.hsubplot = {3, 2, [2 4]};
    show.deltasubplot = {3, 2, [5 6]};
     % customize labels
    labels.xdelta = labels.iter;
    labels.deltalegend = labels.deltatheta;
    gtitle = sprintf('Least-squares policy iteration, ell=%d', cfg.ell);
elseif cfg.lspionlinestep || cfg.lspihonlinestep || cfg.mpcstep,
    show.world = 1;
    show.x = X(:, cfg.k+1, cfg.trial);
    if cfg.lspionlinestep,          algname = 'Online LSPI';
    elseif cfg.lspihonlinestep,     
        if isempty(expcfg.con),     algname = 'Online LSPI with explicit h';
        else                        algname = 'Online LSPI with PK';
        end;
    elseif cfg.mpcstep,    algname = 'Online planning';
    end;
    gtitle = sprintf('%s, trial %d, time=%.1fs', algname, cfg.trial, cfg.ktotal*model.Ts);
    show.worldsubplot = {1, 1, 1};
elseif cfg.approxqlearnstep || cfg.approxqlearntrial,
    show.world = 1;
    show.x = X(:, cfg.k+1, cfg.trial);
    algname = 'Approx Q-learning';
    gtitle = sprintf('%s, trial %d, time=%.0fs', algname, cfg.trial, cfg.ktotal*model.Ts);
    if cfg.approxqlearntrial,
        % compute returns
        returns = zeros(cfg.trial, 1);
        for trial = 1:cfg.trial,
            K = find(~isnan(R(1, :, trial)), 1, 'last');
            if K == 1, K = expcfg.maxsteps+1; end;
            returns(trial) = sum((expcfg.gamma .^ (0:K-2)) .* R(1, 2:K, trial));
        end;
        show.return = returns; clear returns;
    end;
    show.worldsubplot = {3, 1, [1 2]};
    show.returnsubplot = {3, 1, 3};
else
    show.nothing = 1;       % signal nothing is in fact being shown
end;

% create or retrieve the view
if isempty(cfg.gview);  
    create = 1; gview = struct; gview.figh = figure; setfigprop(cfg);
    colormap(sty.cm); 
    gview.title = annotation('textbox', [.05 .93 .5 .05], 'String', gtitle, sty.title{:}) ;
    % open movie if needed
    if strcmp(cfg.plottarget, 'movie'),
        avifilename = [cfg.savedir cfg.savefig '.avi'];
        if exist(avifilename, 'file'), delete(avifilename); end;
        gview.aviobj = avifile(avifilename, 'compression', cfg.compression, 'quality', cfg.quality, 'fps', cfg.fps);
    end;
else
    create = 0; gview = cfg.gview; figure(gview.figh);
end;

% ===================================
% ==== Create world if needed =======
if ~isempty(show.world) && create,
    figure(gview.figh);
    subplot(show.worldsubplot{:});
    % draw circle and bullet representing current state
    rectangle('Position', [-1 -1 2 2], 'Curvature', [1,1]);
    gview.bullet = rectangle('Position', [-1 -1 2 2] * sty.bulletsize, 'Curvature', [1,1]);
    set(gview.bullet, 'EdgeColor', 'none', 'FaceColor', sty.bulletcolor);
    % draw ticks
    for ta = -180+sty.tickspacing : sty.tickspacing :180,
        tarad = ta * pi/180;
        hl = line([.75 .88] * cos(tarad), [.75 .88] * sin(tarad));
        set(hl, 'Color', sty.tickcolor);
    end;
    hmt = patch([-1 0 1] * sty.mainticksize, .88 - [2 0 2] * sty.mainticksize, sty.maintickcolor);
    set(hmt, 'EdgeColor', 'none');
    xlim([-1-sty.bulletsize 1+sty.bulletsize]);
    ylim([-1-sty.bulletsize 1+sty.bulletsize]);
    axis square off manual
    drawnow;
end;

% ===================================================
% ==== Perform the update of the graphical view =====

if ~isempty(show.x),
    subplot(show.worldsubplot{:});
    % show agent at updated (next) position
    pos = [cos(show.x(1) + pi/2) sin(show.x(1) + pi/2)];
    set(gview.bullet, 'Position', [pos 0 0] + [-1 -1 2 2] * sty.bulletsize);
end;

% change the title if needed
if ~show.nothing,
    set(gview.title, 'String', gtitle);
end;

if ~isempty(show.Q),
    subplot(show.Qsubplot{:});
    mesh(pgrid.vertices{1}, pgrid.vertices{2}, show.Q');
    xlabel(labels.x{1}); ylabel(labels.x{2}); zlabel(labels.Q);
    xlim([-model.maxx(1) model.maxx(1)]); ylim([-model.maxx(2) model.maxx(2)]);
    view([-28 42]);
end;

if ~isempty(show.h),
    subplot(show.hsubplot{:});
    ph = pcolor(pgrid.vertices{1}, pgrid.vertices{2}, show.h');
    set(ph, 'LineStyle', 'none'); hold on; 
    xlabel(labels.x{1}); ylabel(labels.x{2}); title(labels.h);
    xlim([-model.maxx(1) model.maxx(1)]); ylim([-model.maxx(2) model.maxx(2)]);
    box on;
end;

if ~isempty(show.delta),
    subplot(show.deltasubplot{:});
    plot(show.deltaxaxis, show.delta, sty.delta{:});
    xlabel(labels.xdelta); 
    legend(labels.deltalegend, 'Location', 'NorthEast');
end;

if ~isempty(show.return),
    subplot(show.returnsubplot{:});
    stairs(1:cfg.trial, show.return, sty.return{:});
    xlabel(labels.trial); 
    ylabel(labels.return);
end;
% ==========================================
% ==== Save graphical output (if any) ======

if ~show.nothing,
    switch cfg.plottarget,
        case 'screen',
            drawnow;
            if isscalar(cfg.pause), 
                if cfg.pause >= 0, pause(cfg.pause),
                else pause;
                end;
            else
                % assuming cell with structure 
                % {condition pause; condition pause; ...}; should include at
                % least a default where the condition evaluates true
                for i = 1:size(cfg.pause, 1),
                    if eval(cfg.pause{i, 1}),
                        if cfg.pause{i, 2} >= 0, pause(cfg.pause{i, 2}),
                        else pause;
                        end;
                        break;
                    end;
                    % otherwise try the next condition
                end;
            end;
        case 'movie',
            % add the figure as a frame to the movie
            drawnow;
            gview.aviobj = addframe(gview.aviobj, gview.figh);
            % determine if we need to take a snapshot at this stage
            if ~isempty(cfg.snapshot),
                if length(cfg.snapshot) == 1,
                    takesnapshot = ((cfg.fuzzyqiter || cfg.lspiter) && (cfg.ell == cfg.snapshot));
                elseif length(cfg.snapshot) == 2,
                    takesnapshot = ((cfg.lspionlinestep || cfg.lspihonlinestep || cfg.mpcstep) ...
                        && (cfg.trial == cfg.snapshot(1)) && (cfg.k == cfg.snapshot(2)));
                else
                    takesnapshot = 0;
                end;
            else
                takesnapshot = 0;
            end;
            if takesnapshot,
                % disp('snapshot');
                saveplot(gview.figh, [cfg.savedir cfg.savefig], cfg.snapshottarget);
            end;
    end;
end;

% Finalize movie if requested to do so
if cfg.finalize,
    if strcmp(cfg.plottarget, 'movie'),
        gview.aviobj = close(gview.aviobj);
    end;
end;

varargout = {gview.figh, gview};

end
% END =================================================

