function approx_ploth(app, theta, xb, ub, cfg)
% Generic policy plot for an approximator (works for Q-function as well as explicit policy
% approximators)
CFG.npoints = 30;
CFG.slice = [];
CFG.uix = 1;        % which of the variables to plot
CFG.xb = [];        % override x bounds
CFG.ub = [];        % override u bounds
CFG = setfigprop(CFG, 'addfields');  % add figure settings fields to CFG
if nargin < 5, cfg = []; end;
cfg = parseconfig(cfg, CFG);
app = revise_approx(app);

% apply any bound overrides
if ~isempty(cfg.xb), xb = cfg.xb; end;
if ~isempty(cfg.ub), ub = cfg.ub; end;

p = size(xb, 1);
q = size(ub, 1);
if p == 2 && q == 1,    
    % fast code for 2 x and 1 u var
    x = {xb(1, 1):diff(xb(1, :))/(cfg.npoints-1):xb(1, 2), ...
        xb(2, 1):diff(xb(2, :))/(cfg.npoints-1):xb(2, 2)};
    xf = flat(x);
    if app.vectorized,  
        h = reshape(app.h(app, theta, xf), length(x{1}), length(x{2}));
    else
        h = zeros(length(x{1}), length(x{2})); 
        for i = 1:size(xf, 2), 
            h(i) = app.h(app, theta, xf(:, i));
        end;
    end;
    ph = pcolor(x{1}, x{2}, h'); set(ph, 'EdgeColor', 'none');
    setfigprop(cfg);
else
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
    % compute h, plot slice
    if app.vectorized,
        h = app.h(app, theta, gr.points);
        h = h(cfg.uix, :);  % select variable
    else
        h = zeros(gr.npoints, 1);
        for i = 1:gr.npoints,
            u = app.h(app, theta, gr.points(:, i)); 
            h(i) = u(cfg.uix); % select variable
        end;
    end;
    ph = pcolor(gr.vertices{expandvars(1)}, gr.vertices{expandvars(2)}, ...
        reshape(h, gr.size)');
    set(ph, 'EdgeColor', 'none');
    setfigprop(cfg);
end;