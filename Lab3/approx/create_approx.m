function approx = create_approx(model, cfg)
% Generic function to create approximator (both Q-function and policy)
% Forwards the call to the appropriate construction function according to the approximator
% type. See respective approximator functions for details
% Provides an "enhanced" mode where instead of requiring explicit parameters of the
% approximators, it creates them automatically. Supports for instance equidistant and
% logarithmic grids for the discrete action or the centers of the basis functions, and various
% ways of computing RBF radii.

% main config
CFG.type = '';          
CFG.enhanced = 0;       % enhanced mode
% enhanced mode fields (ignored if enhanced == 0)
% CFG.problem = '';
CFG.N = [];             % numbers of fuzzy cores/RBF centers on each axis
CFG.M = [];             % numbers of discrete actions on each axis / order of poly approximator in u
    % both N or M can be a vector of size model.p/model.q, or a scalar, in the first case different
    % #s of BFs are used on each axis; in the latter case, the same number is used for all
    % axes
    % M has no effect when uspace = 'pre' (see below)
CFG.xspace = 'eq';      % 'eq'uidistant, 'log'arithmic
CFG.uspace = 'log';     % 'eq'uidistant, 'log'arithmic, or 'pre'defined
CFG.rbfrad = 'avg';     % 'avg' average distance to neigbors; 'max' max dist to neigh; vector = identical radii for all RBFs
CFG.U = [];             % predefined actions space if uspace='pre'; note only supports cell array of grids
% fields below no longer used
CFG.thresh = [];        % threshold for RBFs (if any)
CFG.ucon = [];          % use constraints for the control action; should be empty or function handle
CFG.uconinitargs = {};  % arguments of ucon function in INIT mode

SUPPORTEDTYPES = {'triang', 'rbfdisc', 'triangquad', 'rbfquad', ...
    'rbfpoly', 'triangpoly', 'rbfortho', 'triangortho', ...
    'rbfsing', 'triangsing', 'rbflindisc'};

% also support single, flat-argument call with approximator type
if ischar(cfg) && any(strcmp(cfg, SUPPORTEDTYPES)),
    type = cfg; cfg = struct; cfg.type = type;
end;

cfg = parseconfig(cfg, CFG);

if ~isempty(cfg.thresh) || ~isempty(cfg.ucon) || ~isempty(cfg.uconinitargs),
    error('Threshold and ucon support have been removed');
end;

if ~cfg.enhanced,
    % basic mode
    % forward call to appropriate function
    switch cfg.type,
        case SUPPORTEDTYPES,
            approx = feval(cfg.type, model, cfg);
        otherwise,
            error(['Unknown approximator type [' cfg.type ']']);
    end;
else
    % enhanced mode
    
    % form X and possibly U grids (if required by approximator ty
    xgrids = cell(model.p, 1); 
    makeugrids = any(strcmp(cfg.type, {'triang', 'rbfdisc', 'rbflindisc'})) ...
        && ~strcmp(cfg.uspace, 'pre');
    if makeugrids, ugrids = cell(model.q, 1); end;
    for i = 1:model.p+(makeugrids * model.q),
        if i<=model.p,  
            sp = cfg.xspace; b = model.maxx(i); 
            if isscalar(cfg.N),     n = cfg.N;
            else                    n = cfg.N(i);
            end;
        else
            sp = cfg.uspace; b = model.maxu(i-model.p); 
            if isscalar(cfg.M),     n = cfg.M;
            else                    n = cfg.M(i-model.p);
            end;
        end;
        switch sp,
            case 'eq',
                gr = -b : (2*b/(n-1)) : b;
            case 'log',
                gr = symloggrid(n, b);
        end;
        if i<=model.p,  xgrids{i} = gr;
        else            ugrids{i-model.p} = gr;
        end;
    end;
    
    if strcmp(cfg.uspace, 'pre'),
        if ~iscell(cfg.U), error('Predefined action space should be a cell array of grids'); end;
        ugrids = cfg.U;
    end;

    % approx config
    acfg = cfg;
    switch cfg.type,
        case 'triang',
            acfg.xgrids = xgrids; acfg.ugrids = ugrids;
        case {'triangquad', 'triangpoly', 'triangortho', 'triangsing'},
            acfg.xgrids = xgrids;
        case {'rbfdisc', 'rbfquad', 'rbfpoly', 'rbfortho', 'rbfsing', 'rbflindisc'},    % form RBFs
            acfg.c = flat(xgrids);
            if any(strcmp(cfg.type, {'rbfdisc', 'rbflindisc'})), acfg.ugrids = ugrids; end;
            % form radii
            if isnumeric(cfg.rbfrad),
                if all(size(cfg.rbfrad) == size(acfg.c)),
                    % radii explicitly specified
                    acfg.rad = cfg.rbfrad;
                elseif length(cfg.rbfrad) == model.p,   
                    % uniform radii
                    acfg.rad = repmat(cfg.rbfrad(:), 1, size(acfg.c, 2));
                end;
            else
                slash = strfind(cfg.rbfrad, '/');
                if ~isempty(slash), divisor = str2num(cfg.rbfrad(slash+1:end));
                else divisor = 1; 
                end;
                rad = cell(model.p, 1);
                for i = 1:model.p,
                    r = diff(xgrids{i}); r = [r(1) r r(end)]; 
                    switch cfg.rbfrad(1:3),
                        case 'avg', r = (r(1:end-1) + r(2:end)) ./ (2 * divisor);
                        case 'max', r = max(r(1:end-1), r(2:end)) ./ divisor;
                    end;
                    rad{i} = r;
                end;
                acfg.rad = flat(rad);
            end;    % if rbfrad is numeric
    end;
    approx = feval(acfg.type, model, acfg);
end;
