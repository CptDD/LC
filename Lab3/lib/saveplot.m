function saveplot(figh, savepath, plottarget)
% Saves figure FIGH to SAVEPATH according to PLOTTARGET specification
%   SAVEPLOT(FIGH, SAVEPATH, PLOTTARGET)

% 2009-12: added 'PaperPositionMode', 'auto'. Note however that this switch
% puts margins differently than manual save! So latex trimming that was customized for 
% manually-exported images may produce undesired results such as clipped
% images

if isempty(figh), figh = gcf; end;

% return;

if verLessThan('matlab', '8.4'), fign = gcf; else, fign = get(gcf, 'Number'); end;

switch plottarget,
    case 'latex',        
        saveas(figh, [savepath '.fig'], 'fig');
        set(figh, 'PaperPositionMode', 'auto');
%         saveas(figh, [savepath '.eps'], 'eps2');
       % maybe add explicit resolution
        print(['-f' num2str(fign)], '-dpsc2', [savepath '.eps']);
    case 'beamer',
        saveas(figh, [savepath '.fig'], 'fig');
        set(figh, 'PaperPositionMode', 'auto');
        % override resolution for PNGs
        print(['-f' num2str(fign)], '-dpng', '-r300', [savepath '.png']);
%         saveas(figh, [savepath '.png'], 'png');
    case 'fig',             % figure only
        saveas(figh, [savepath '.fig'], 'fig');        
    case 'png',             % PNG bitmap only
        print(['-f' num2str(fign)], '-dpng', '-r300', [savepath '.png']);
    case {'screen', ''},  % do nothing
    otherwise,      % do nothing
end;
