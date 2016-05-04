function out = dc_problem(what, varargin)
% DC motor problem setup.
%   OUT = DC_PROBLEM(WHAT)
% This function conforms to the specifications established by SAMPLE_PROBLEM.
%
% Signature for 'model' mode:
%   MODEL = DC_PROBLEM('model', [REW, [DYN]])
% where:
%   - REW is a reward type string (notable types: lqr, box, shaping) or more detailed config (see
%   code for commented defaults) 
%   - DYN a system type string: 'rot' where free rotation is allowed by wrapping around the edges; or 'lin'
%   where the position is bounded to [-pi, pi).
% 
% Implements a special 'changerew' mode that changes the reward function of a given model.


% x, u bounds
maxx = [pi; 16*pi];
maxu = 10;
% default initial state: random
x0 = (2*rand) * maxx - maxx; tend = 1.5;    % should be enough for a good policy
% default gamma
gamma = 0.95;
% defaults for LQR (NOTE only these are used when changing rewards)
Qrew = diag([5, 0.01]); Rrew = 0.01;

switch what
    case 'info',    % basic info about the model (without creating the actual model)
        info.id = 'dc'; info.problem = @dc_problem;
        info.det = 1; info.p = 2; info.q = 1;
        out = info;
        
    case 'shortid',
        out = 'dc';
        
    case 'model'
        % ======== a) 'set in stone' model fields
        model.id = 'dc'; model.problem = @dc_problem;
        model.p = 2;
        model.q = 1;
        model.gamma = gamma;
        model.visualizefun = @dc_ip_visualize;
        model.plotfun = @dc_plot;
        model.userefmodel = 0;      % cannot use a ref model in current implementation

        % ======== b) configurable model fields
        PAR.Ts = 0.005;
        PAR.maxx = maxx;
        PAR.maxu = maxu;
        PAR.type = 'rot';          % default type is rotational to keep backward compatibility
        % stochastic options: either "unreliable actuator"
        PAR.stoch_prob = [];       % if stochastic: probabilities with which...
        PAR.stoch_gain = [];       % the action variable is multiplied with these values
        % or distribution
        PAR.noise = [];            % 'gauss' supported
        PAR.gausssigma = [];       % covariance matrix for gauss noise
        % physical motor parameters
        PAR.J = 3.24e-5;            % rotor and load inertia [Nm2]
        PAR.b = 3.0e-6;             % viscous damping [Nms/rad]
        PAR.K = 53.6e-3;            % torque constant [Nm/A]
        PAR.Re = 9.50;               % rotor resistance [Ohm]
        PAR.L = 0.84e-3;            % rotor inductance [H]
        
        % THIRD argument: empty, model type, or config based on defaults above
        if nargin < 3 || isempty(varargin{2}), 
            par = PAR; par.type = 'rot'; 
        elseif ischar(varargin{2}) && any(strcmp(varargin{2}, {'lin', 'rot'}))
            par = PAR; par.type = varargin{2};
        else    % assumed config structure or string
            par = parseconfig(varargin{2}, PAR);
        end;
        % copy parameters to model structure
        model = copyfields(par, model, fieldnames(par));    % copy params to the model
        
        % compute linear second-order model
        A = [0 1; 0 -model.b/model.J-(model.K^2)/(model.Re*model.J)]; 
        B = [0; model.K/(model.Re*model.J)];
        C = [1 0];
        D = 0;
        G = ss(A,B,C,D);        % state-space model 
        G = c2d(G, model.Ts, 'zoh');  % discrete-time model
        % don't save the SS model as it can create problems
        % when changing Matlab versions
        % model.G = G;      
        model.A = G.a;
        model.B = G.b;
        
        % set dynamics according to type
        switch model.type,
            case 'lin',
                model.fun = @dc_linmdp;     % linear model w/ saturation
            case 'rot',
                model.fun = @dc_mdp;        % rotational model
        end;
        
        % process stochastic model
        model.det = isempty(model.stoch_prob) && isempty(model.noise);
        if ~isempty(model.stoch_prob) && ~isempty(model.noise),
            error('Does not support transition and actuator noise simultaneously'); 
        end;
        if ~isempty(model.stoch_prob), 
            model = stochact_mdpwrapper(model, 'prepare'); 
        end;
        if ~isempty(model.noise),
            model.chol = chol(model.gausssigma);
        end;
        
        % ======== c) reward specification (SECOND argument)
        REW.rewtype = 'lqr';
        REW.Q = Qrew; REW.R = Rrew;                         % for LQR
        REW.cshap = 10; REW.qshap = [pi/4; 4*pi];           % for SHAPING
        REW.bandshap = [pi/4; 4*pi];
        REW.zeroreward = 1; REW.zeroband = [10*pi/180; 0.3]; % for BOX
        REW.sigma = diag([model.maxx'./[3 2] model.maxu/3]);               % for GAUSS
        % two bands used in some experiments:
%         rewband = [5*pi/180 0.2]';    % small band
%         bigrewband = [15*pi/180 0.4]';% big band
        if nargin < 2 || isempty(varargin{1}),  % use defaults
            rew = REW;
        else
            rew = varargin{1};
            % support reward specified simply by type
            if any(strcmp(rew, {'lqr', 'nlqr', 'rot', 'gauss', 'shaping', 'shapbox', 'box', 'BigBox'})),
                rtype = rew;
                rew = REW;
                rew.rewtype = rtype; % the rest remain at defaults
            else    % full-fledged configuration parsing
                rew = parseconfig(varargin{1}, REW);
                if numel(rew.Q) == 2,   % specfied as vector of diagonal elements
                    rew.Q = diag(rew.Q);
                end;
            end;
        end;
        switch rew.rewtype,
            case 'lqr',
                % LQR
                model = copyfields(rew, model, {'rewtype', 'Q', 'R'});
                model.maxr = maxx' * model.Q * maxx + model.R * maxu * maxu;
            case 'nlqr',
                % normalized LQR
                model = copyfields(rew, model, {'rewtype', 'Q', 'R'});
                model.nonnorm_maxr = model.maxx' * model.Q * model.maxx + model.R * model.maxu * model.maxu;
                model.maxr = 1;
            case 'gauss',
                model = copyfields(rew, model, {'rewtype', 'sigma'});
                model.W = inv(2 * model.sigma * model.sigma);
                model.maxr = abs(-1 + exp(-[model.maxx; model.maxu]' * model.W * [model.maxx; model.maxu]));
                % to run a fuzzyQI exper
                % clear fc; fc.run = 1; fc.model_params={'gauss', 'lin'}; fc.datafile='dcfz_fine_gauss'; fc.problem='dc_problem'; fuzzyqi(fc);
            case 'shaping',
                % LQR + Manhattan shaping
                model = copyfields(rew, model, {'rewtype', 'Q', 'R', 'cshap', 'qshap'});
                % upper bound on reward, considering the shaping
                model.maxr = maxx' * model.Q * maxx + model.R * maxu * maxu ...
                    + model.cshap * sum(floor(maxx(:) ./ model.qshap));
            case 'shapbox',
                % LQR + box shaping
                model = copyfields(rew, model, {'rewtype', 'Q', 'R', 'cshap', 'bandshap'});
                % upper bound on reward, considering the shaping
                model.maxr = maxx' * model.Q * maxx + model.R * maxu * maxu + model.cshap;
            case {'box', 'BigBox'},
                % box reward
                model = copyfields(rew, model, {'rewtype', 'zeroband', 'zeroreward'});
                model.maxr = model.zeroreward;
                % to run a fuzzyQI exper
                % clear fc; fc.run = 1; fc.model_params={'box', 'lin'}; fc.datafile='dcfz_fine_box'; fc.problem='dc_problem'; fuzzyqi(fc);
        end;

        out = model;
        
    case 'reset',   % handle state reset
        RES.type = [];
        RES.x0 = [];
        model = varargin{1};
        res = varargin{2};
        if ischar(res) && any(strcmp(res, {'rand'})),   % handle simple method identifier
            res = struct('type', res);
        elseif isnumeric(res),
            res = struct('type', 'values', 'x0', res);
        end;
        
        res = parseconfig(res, RES);
        switch res.type,
            case 'rand',
                x0 = (2*rand(model.p, 1) - 1) .* model.maxx;
            case 'values',
                if size(res.x0, 2) > 1,    x0 = res.x0(:, unidrnd(size(res.x0, 2)));
                else                       x0 = res.x0;
                end;
        end;                    
        out = x0;

    case 'X0',
        if nargin < 2, xt = 'fine';         % Default is fine scale
        else xt = varargin{1}; end;
        switch xt,
            case 'coarse',
                out = {-pi:30*pi/180:pi, -maxx(2):2*pi:maxx(2)};
            case 'fine',
                out = {-pi:10*pi/180:pi, -maxx(2):pi:maxx(2)};
            case 'small',
                out = {-pi:pi/2:pi, [-10 -5 -2 -1 0 1 2 5 10]*pi};
            case 'small2',
                out = {-pi:pi/3:pi, -maxx(2):4*pi:maxx(2)};
            otherwise,
                error(['Unknown X0 type [' xt ']']);
        end;
        
    case 'fuzzy'
        cfg.gamma = gamma;
        
        % process arguments for fuzzy grids
        if nargin < 2, xg = 'fine';         % Default is fine scale
        else xg = varargin{1}; end;
        if nargin < 3, Npoints = 20;        % Default 20 log points on each side of each axis (including 0)
        else Npoints = varargin{2}; end;
        
        switch xg,
            case 'fine',        % use fine state grids
                cfg.xgrids = {-pi:5*pi/180:pi, -maxx(2):pi/6:maxx(2)};
            case 'log',
                % generate logarithmically spaced points between 0 and 1
                ls = (logspace(-1, 0, Npoints) - 0.1) * 1/0.9;
                % scale them and replicate to left and right
                cfg.xgrids = {sortx([-maxx(1)*ls maxx(1)*ls]), sortx([-maxx(2)*ls maxx(2)*ls])};
        end;
        cfg.ugrids = {[-10, -5, -1, -0.1, 0, 0.1, 1, 5, 10]};
        % ugrids = {-maxu:1:maxu};      % this is for bang-bang grid
        cfg.x0 = x0;
    
        out = cfg;
        
    case {'lspi', 'lspe'}
        cfg.gamma = gamma;
        cfg.x0 = x0;
        cfg.tend = tend;
        out = cfg;
        
    case 'fittedqi';
        cfg.gamma = gamma;
        out = cfg;        

    case 'grid'
%         cfg.xgrids = xgrids;
%         cfg.ugrids = ugrids;
        cfg.gamma = gamma;
        cfg.x0 = x0;
        out = cfg;
    
    case 'changerew',       % change reward function & maxr accordingly
        model = varargin{1};
        model.rewtype = varargin{2};
        switch model.rewtype,
            case 'lqr',
                % config 2: LQR
                model.Q = Qrew;
                model.R = Rrew;
                model.maxr = maxx' * model.Q * maxx + model.R * maxu * maxu;
            otherwise
                error(['Cannot change reward to [' model.rewtype ']']);
        end;
        out = model;
        
end;


% END doubleint_problem(), RETURNING out ====================================