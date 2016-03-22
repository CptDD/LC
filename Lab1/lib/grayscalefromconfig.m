% Obtains grayscale value from config fields for various plotting utilities
function grayscale = grayscalefromconfig(cfg)

if isfield(cfg, 'plottarget'),
    switch cfg.plottarget,
        case 'latex',                       grayscale = 1;
        case {'beamer', 'screen', ''},      grayscale = 0;
        otherwise,                          grayscale = 1;      % safe default
    end;
else grayscale = 1;     % safe default
end;
