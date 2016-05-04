function approx_plotv(app, theta, xb, ub, cfg)
% Generic value-function plot for an approximator
CFG.npoints = 30;
CFG.slice = [];     % default slice will be [NaN, NaN, 0, 0, ..., 0]
CFG.xb = [];        % override for x bounds; use to focus on certain parts of X
CFG.ub = [];        % override for u bounds; use to focus on certain parts of U
CFG.mesh = {};      % mesh properties (useful for plot functions, e.g., dc_plot, bike_plot)
CFG = setfigprop(CFG, 'addfields');  % add figure settings fields to CFG
if nargin < 5, cfg = []; end;
cfg = parseconfig(cfg, CFG);

% two-stage call to compute v(qstar) could be made 1-stage (with the
% collaboration of the approximator functions)

% apply any bound overrides
if ~isempty(cfg.xb), xb = cfg.xb; end;
if ~isempty(cfg.ub), ub = cfg.ub; end;

p = size(xb, 1);
if p == 2,  
    % fast code for two-dimensional state space
    x = {xb(1, 1):diff(xb(1, :))/(cfg.npoints-1):xb(1, 2), ...
        xb(2, 1):diff(xb(2, :))/(cfg.npoints-1):xb(2, 2)};
    xf = flat(x);
    if isfield(app, 'vectorized') && app.vectorized,
        % u = app.h(app, theta, xf); v = app.q(app, theta, xf, u);
        % above less efficient; below assumes that app.h returns maximal Q-values:
        [u, v] = app.h(app, theta, xf);
        v = reshape(v, length(x{1}), length(x{2}));
    else
        v = zeros(length(x{1}), length(x{2})); 
        for i = 1:size(xf, 2), 
            % V is the Q of the best action (not very efficient...)
            u = app.h(app, theta, xf(:, i)); 
            v(i) = app.q(app, theta, xf(:, i), u);
        end;
    end;
    mesh(x{1}, x{2}, v', cfg.mesh{:});
    setfigprop(cfg);
elseif p > 2,
    % general code for n dimensions
    if isempty(cfg.slice),  % default slice is first 2 variables, the rest=0
        cfg.slice = [NaN NaN zeros(1, p-2)];
    end;
    if sum(isnan(cfg.slice)) ~= 2,
        error('Can only expand precisely 2 variables (#NaNs == 2)');
    end;
    % make grid
    expandvars = find(isnan(cfg.slice));    % expand these vars
    gr.vertices = cell(p, 1);
    gr.size = [];
    for i = 1:p,
        if any(i == expandvars),    % expanded var
            gr.vertices{i} = xb(i, 1):diff(xb(i, :))/(cfg.npoints-1):xb(i, 2);
            gr.size(end+1) = length(gr.vertices{i});
        else                        % collapsed var
            gr.vertices{i} = cfg.slice(i);
        end;
    end;
    gr.points = flat(gr.vertices);
    gr.npoints = size(gr.points, 2);
    % compute V function, plot slice
    if isfield(app, 'vectorized') && app.vectorized,
        % u = app.h(app, theta, gr.points); 
        % v = app.v(app, theta, gr.points, u);
        % above less efficient; below assumes that app.h returns maximal Q-values:
        [u, v] = app.h(app, theta, gr.points);
    else
        v = zeros(gr.npoints, 1);
        for i = 1:gr.npoints,
            % this code not very efficient...
            u = app.h(app, theta, gr.points(:, i)); 
            v(i) = app.q(app, theta, gr.points(:, i), u);
        end;
    end;
    mesh(gr.vertices{expandvars(1)}, gr.vertices{expandvars(2)}, ...
        reshape(v, gr.size)', cfg.mesh{:});
    setfigprop(cfg);
end;

