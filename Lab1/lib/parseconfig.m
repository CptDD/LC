function [cfg, overridden, defaulted] = parseconfig(cfg, CFG, ECFG, mode)
% Parse and process typical options in configuration string or structure
% Parameters:
%   cfg     - configuration to process
%   CFG     - defaults
%   ECFG    - early defaults
%   mode    - mode in which the problem function should be called, if any
% Returns:
%   cfg         - the parsed configuration
%   overridden  - fields that were explicitly set
%   defaulted   - fields that defaulted

if nargin < 3, ECFG = struct; end;
if nargin < 4, mode = []; end;

% make sure cfg and ECFG is struct even if empty
if isempty(cfg), cfg = struct; end;
if isempty(ECFG), ECFG = struct; end;
    
% if provided string, parse it into a structure
if ischar(cfg), cfg = str2cfg(cfg, [fieldnames(CFG); fieldnames(ECFG)]); end;
% install early defaults
[cfg, flag, overridden, defaulted] = checkparams(cfg, ECFG);
% call problem if a problem field exists & mode is specified
if isfield(cfg, 'problem') && ~isempty(cfg.problem) && ~isempty(mode),
    if isfield(cfg, [mode '_params']),
        % use problem mode params if available on the config
        [cfg, flag, o2, d2] = checkparams(cfg, ...
            feval(cfg.problem, mode, cfg.([mode '_params']){:})); 
    else
        [cfg, flag, o2, d2] = checkparams(cfg, feval(cfg.problem, mode)); 
    end;
    % append new field names
    overridden = {overridden{:}, o2{:}}; defaulted = {defaulted{:}, d2{:}};
end;
% install defaults for whatever hasn't been specified up to now
[cfg, flag, o2, d2] = checkparams(cfg, CFG);
% append new field names
overridden = {overridden{:}, o2{:}}; defaulted = {defaulted{:}, d2{:}};

% process typical configuration dependencies
if isfield(cfg, 'silent') && isfield(cfg,'verb') && cfg.silent, 
    cfg.verb = -Inf; 
end;
if isfield(cfg, 'grayscale'),
    cfg.grayscale = grayscalefromconfig(cfg);
end;
