%% Some initialization
% create a dummy model to have access to the state and action space bounds (in maxx, maxu)
dummymodel = ipsetup_problem('model');

% Note you can run this file in cell mode; each cell is separated by double comment signs
% CTRL+ENTER inside a cell runs that cell; CTRL+UP or DOWN steps one cell
% up or down

%% Fuzzy Q-iteration: configure and run
N = 21;         % number of interpolation points on the state grid, for each axis
M = 5;          % number of discretization points on the action grid
fzcfg = struct;
fzcfg.problem = 'ipsetup_problem';
fzcfg.gamma = 0.98;             % discount factor
fzcfg.eps = 0.1;                % convergence threshold
% state space approximator: an equidistant interpolation grid with N x N elements
% (the function symequidgrid creates this grid)
fzcfg.xgrids = symequidgrid(N, dummymodel.maxx);
% action space approximator: an equidistant discretization grid with M elements
fzcfg.ugrids = {symequidgrid(M, dummymodel.maxu)};
fzcfg.datadir = pwd;            % save data to the current directory (pwd)
fzcfg.datafile = 'ip_fzqi';     % save to a data file having this name
fzcfg.itersave = 1e10;          % save only at the end to avoid spurious warnings in recent Matlab versions
% uncomment this code if you wish to visualize the solution while computing it 
% (the algorithm will run much slower due to this)
% fzcfg.visualize = 1; fzcfg.savetheta = 1; ccfg.viscfg.figsize = [800 500]; fzcfg.itervis = 10;

% run the algorithm, remembering the statistics
fzcfg.run = 1;
[thetastar, qistats] = fuzzyqi(fzcfg);                 

% qistats now contains a structure with the statistics of the run;
% importantly, the total execution time is
exectime = qistats.tinit + qistats.t
% (that is, initialization time + time taken to run the iterations)


%% Fuzzy Q-iteration: Extract solution in the form of a standard "approximator" structure, and examine it
fzcfg = struct;
fzcfg.approx = 1;                % call in mode "approx"
fzcfg.datafile = 'ip_fzqi';      % for the datafile used before
[fzapprox, fztheta] = fuzzyqi(fzcfg);

% plot the maximum Q-function (value function) and the policy
% note the use of the "plotv" and "ploth" functions supplied by the approximator structure
figure; set(gcf, 'position', [0, 0, 800,400]); movegui(gcf, 'center');
subplot(121); fzapprox.plotv(fzapprox, fztheta); xlabel('alpha'); ylabel('alpha'''); zlabel('max_u Q(x, u)');
subplot(122); fzapprox.ploth(fzapprox, fztheta); xlabel('alpha'); ylabel('alpha'''); title('h(x)');

% replay a trajectory from close to the "pointing down" initial state, 4-seconds long, 
% with the solution just computed
% Note we start close to and not exactly at -pi, because often the
% approximate solutions will lead to action 0 at exactly -pi, so we would
% get an identically zero action solution that is not representative of the
% overal quality of the aproximator
fzhist = fuzzyqi('replay x0=[-0.95*pi,0] tend=4 datafile=ip_fzqi');
% extract the return R(x0)
fzret = fzhist.R(1);