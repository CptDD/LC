function t = triang(model, cfg)
% Create triangular fuzzy state approximator with discrete actions

CFG.type = 'triang';            % # of samples to generate
CFG.disc = 1;
CFG.xgrids = [];
CFG.ugrids = [];
CFG.fuzzy_params = {};          % parameters to call problem in fuzzy mode

cfg = parseconfig(cfg, CFG);

if isempty(cfg.xgrids) || isempty(cfg.ugrids),
    % need to obtain grids from the problem
    fz = feval(model.problem, 'fuzzy', cfg.fuzzy_params{:});
    if isempty(cfg.xgrids), cfg.xgrids = fz.xgrids; end;
    if isempty(cfg.ugrids), cfg.ugrids = fz.ugrids; end;
end;
if isempty(cfg.xgrids) || isempty(cfg.ugrids),
    error('X and/or U grids could not be obtained');
end;

    
t.type = cfg.type;
t.sparse = 1;       % recommend usage of sparse activation vectors
t.extendedsyntax = 1;
% core functions
t.phi = @triang_phi; t.q = @triang_q; t.h = @triang_h; 
% plot functions
t.plotv = @triang_plotv; t.ploth = @triang_ploth;
% centers grids
t.c = cfg.xgrids;
% helper vars for mdegs_c
t.p = model.p;
t.Nc = []; for i = length(t.c):-1:1, t.Nc(i) = length(t.c{i}); end;
t.tab = dec2base(0:2^model.p-1, 2) - 47;
t.roll = 0*t.Nc;
% total number of MFs
t.Ncflat = prod(t.Nc);
% action space, structured and flat
t.U = cfg.ugrids;
t.Uflat = flat(cfg.ugrids);
% number of actions
t.M = size(t.Uflat, 2);
% finally, total number of basis functions
t.N = t.Ncflat * t.M;

% other helper vars
v = repmat(1:t.M, 2^t.p, 1);
t.uoff = (v - 1) .* repmat(t.Ncflat, 2^t.p, t.M);

end     % TRIANG constructor


% Compute BF values in x, u
function phi = triang_phi(t, x, u)
phi = sparse(t.N, 1);
[ind, mu] = mdegs_p(x, t.c, t.roll, t.Nc, t.p, t.tab);
ui = findflat(u, t.Uflat, 1, 'first');
phi((ui-1) * t.Ncflat + ind) = mu;
end

% Compute Q-value in x, u
function [q, phi] = triang_q(t, theta, x, u)
[ind, mu] = mdegs_p(x, t.c, t.roll, t.Nc, t.p, t.tab);
ui = findflat(u, t.Uflat, 1, 'first');
q = mu' * theta((ui-1) * t.Ncflat + ind);
if nargout > 1,
    phi = sparse(t.N, 1);
    phi((ui-1) * t.Ncflat + ind) = mu;
end;
end

% Compute greedy policy in x
function [u, qstar] = triang_h(t, theta, x)
[ind, mu] = mdegs_p(x, t.c, t.roll, t.Nc, t.p, t.tab);
qx = mu' * theta(repmat(ind, 1, t.M) + t.uoff);
qstar = max(qx);
u = t.Uflat(:, find(qx == qstar, 1, 'first'));
end

% ----- Plot functions
% Plot V - forward call to generic plot fun
function triang_plotv(t, theta, varargin)
for i = length(t.c):-1:1, xb(i, 1:2) = t.c{i}([1 end]); end;
for i = length(t.U):-1:1, ub(i, 1:2) = t.U{i}([1 end]); end;
approx_plotv(t, theta, xb, ub, varargin{:});
end
% Plot h - forward call to generic plot fun
function triang_ploth(t, theta, varargin)
for i = length(t.c):-1:1, xb(i, 1:2) = t.c{i}([1 end]); end;
for i = length(t.U):-1:1, ub(i, 1:2) = t.U{i}([1 end]); end;
approx_ploth(t, theta, xb, ub, varargin{:});
end
