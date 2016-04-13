function gview = gridnav_visualize(cfg)
% Visualizes certain objects in a graphical representation ("view") of the grid navigation problem
% The view is created automatically, unless the field 'gview' is set, see below.
% Fields on the configuration 'cfg':
%   gview   - if view already created, this field should contain the view structure returned. This
%       will enable a (computationally cheap) update of the view, rather than its (expensive) recreation
%   model   - the model of the grid navigation MDP
%   Q       - if a Q-function should be represented, this field should contain it, in a
%   3-dimensional array of size gridwidth x gridheight x 4, where 4 is the number of actions
%   The Q-function representation shows max(Q) for a given state as a color shade -- note this in
%   fact represents a V-function, and specifically the optimal V-function when the optimal
%   Q-function is provided
%   h       - if a policy should be represented, this field should contain it, in a matrix of size
%   gridiwdth x gridheight
%   x       - if the robot should be shown in a certain state, this field should contain it as a
%   column vector

% default arguments
if nargin < 1, cfg = ''; end;

CFG.gview = [];           % if nonempty: update existing; if empty: create new; graphical view
% update/create view with:
CFG.model = [];
CFG.Q = [];               % Q-function
CFG.h = [];               % policy
CFG.x = [];               % state

% output configuration
CFG.pause = .1;
CFG = setfigprop(CFG, 'addfields');
% process config
cfg = parseconfig(cfg, CFG);

% pickup model
model = cfg.model;

sty = struct;
% colors / linestyles
sty.title = {'HorizontalAlignment', 'left', 'FontSize', 12, 'FontWeight', 'bold'};
sty.emptyc = [1 1 1];    % empty cell background
sty.obstc = [.3 .3 .3];  % obstacle background
sty.cell = {'LineWidth', 2, 'EdgeColor', [.3 .3 .3]};    % cell style
sty.actionc = [0 0 0];
sty.arrow = {'Color', sty.actionc, 'LineWidth', 3, 'HeadWidth', 15, 'HeadStyle', 'plain'};
% sizing (cell is unit size)
sty.axismargin = .01;   % add this to axis limits around the graphics, to avoid clipping edges
sty.cellmargin = .01;   % add this around cells to allow thicker lines
sty.imgsize = .93;      % object image size
sty.arrowpadding = .1;  % padding of action arrow
% images
fileinfo = what('gridnav'); imgpath = [fileinfo(1).path '/images/'];
sty.img_robot = [imgpath 'robot.png'];
sty.img_goal = [imgpath 'goal.png'];

% ===================================
if isempty(cfg.gview);  
    create = 1; gview = struct; gview.figh = figure;
    % adjust default figure size to grid size (times correction factor) 
    if isempty(cfg.figsize), cfg.figsize = model.size * 80 .* [1.1 1]; end;
    setfigprop(cfg);
else
    create = 0; gview = cfg.gview; figure(gview.figh);
end;

% ===================================
% ==== Create world if needed =======
if create,
    N = model.size; gview.N = N;
    % --- draw cells; note cells are unit size; leave some room around to avoid clipping the edges
    set(gca, 'xlim', [-sty.axismargin N(1)+sty.axismargin], 'ylim', [-sty.axismargin N(2)+sty.axismargin]);
    axis off;
    gview.cells = zeros(N);
    for i = 1:N(1), 
        for j = 1:N(2),
            gview.cells(i, j) = rectangle('Position', ...
                [i-1+sty.cellmargin, j-1+sty.cellmargin, 1-2*sty.cellmargin, 1-2*sty.cellmargin], ...
                sty.cell{:});
            if ~isempty(findflat([i; j], model.x_obst)),  
                set(gview.cells(i, j), 'FaceColor', sty.obstc);
            else
                set(gview.cells(i, j), 'FaceColor', sty.emptyc);
            end;
        end; 
    end;
    % --- draw goal object
    gview.img_goal = image('CData', imread(sty.img_goal), ...
        'XData', model.x_goal(1)-1+.5+[-sty.imgsize/2 sty.imgsize/2], ...
        'YData', model.x_goal(2)-1+.5+[sty.imgsize/2 -sty.imgsize/2]);
    % --- draw policy arrows (but only for nonterminal states)
    gview.arrows = zeros(N(1), N(2), 4); 
    for i = 1:N(1),
        for j = 1:N(2),
            % transform from data coords to figure coords (required to draw the arrow)
            [ax ay] = dsxy2figxy(gca, [i-1+sty.arrowpadding, i-.5, i-sty.arrowpadding], ...
                [j-1+sty.arrowpadding, j-.5, j-sty.arrowpadding]);
            % left, right, down, up
            gview.arrows(i, j, 1) = annotation('arrow', ax([3 1]), ay([2 2]), sty.arrow{:});
            gview.arrows(i, j, 2) = annotation('arrow', ax([1 3]), ay([2 2]), sty.arrow{:});
            gview.arrows(i, j, 3) = annotation('arrow', ax([2 2]), ay([3 1]), sty.arrow{:});
            gview.arrows(i, j, 4) = annotation('arrow', ax([2 2]), ay([1 3]), sty.arrow{:});
        end;
    end;
    set(gview.arrows(:), 'Visible', 'off');     % hide the arrows    
    % --- draw robot images on top of the arrows
    gview.img_robot = image('CData', imread(sty.img_robot), ...
        'XData', .5+[-sty.imgsize/2 sty.imgsize/2], ...
        'YData', .5+[sty.imgsize/2 -sty.imgsize/2]);
    set(gview.img_robot, 'Visible', 'off');    % hide the robot, it's not always needed
end;

% ===================================================
% ==== Perform the update of the graphical view =====

if ~isempty(cfg.x),
    % show agent at updated (next) position
    set(gview.img_robot, 'XData', cfg.x(1)-.5+[-sty.imgsize/2 sty.imgsize/2], ...
        'YData', cfg.x(2)-.5+[sty.imgsize/2 -sty.imgsize/2], 'Visible', 'on');
else set(gview.img_robot, 'Visible', 'off'); 
end;

if ~isempty(cfg.Q),
    V = max(cfg.Q, [], 3);  % across the action variable
    if min(V(:)) ~= max(V(:)), % avoid division by zero
        V = (V - min(V(:))) ./ (max(V(:)) - min(V(:))); % scale in [0, 1]
    else
        V(:) = 0;   % make sure it's in the allowable range
    end;
    colors = winter(100);
    for i = 1:gview.N(1),
        for j = 1:gview.N(2),
            if ~model.Xterminal(i+(j-1)*gview.N(1)),
                set(gview.cells(i, j), 'FaceColor', colors(1+floor(V(i, j)*99), :));
            end;
        end;
    end;
else
    % cleanup
    set(gview.cells(:), 'FaceColor', sty.emptyc);
    if model.x_obst > 0,    % else just a dummy value
        for i = 1:size(model.x_obst, 2),
            set(gview.cells(model.x_obst(1, i), model.x_obst(2, i)), 'FaceColor', sty.obstc);
        end;
    end;
end;

if ~isempty(cfg.h),
    % reset the arrows to their default state
    set(gview.arrows(:), 'Visible', 'off');
    % make left/right arrows visible in states that take the respective action
    for i = 1:gview.N(1),
        for j = 1:gview.N(2),
            if ~model.Xterminal(i+(j-1)*gview.N(1)),
                set(gview.arrows(i, j, cfg.h(i, j)), 'Visible', 'on');
            end;
        end;
    end;
else 
    % cleanup
    set(gview.arrows(:), 'Visible', 'off');
end;

% ==========================================
% ==== Finalize  ======

drawnow;
if cfg.pause >= 0, pause(cfg.pause),
else pause;
end;

end
% END cleanrob_visualize =================================================
