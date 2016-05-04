function approx = revise_approx(approx)
% Revise older approximator structures to conform to new implementation details
% For instance, add a truncation flag where one is missing
% Should be called whenever loading approximator objects from files

if ~isfield(approx, 'vectorized'), approx.vectorized = 0; end;
if ~isfield(approx, 'extendedsyntax'), approx.extendedsyntax = 0; end;  % was 1 before 30 March 2011!

if isfield(approx, 'thresh'), approx = rmfield(approx, 'thresh'); end; % threshold no longer used
if isfield(approx, 'ucon'), approx = rmfield(approx, 'ucon'); end; % no longer used

if ~isfield(approx, 'disc'),        % verify & declare whether using discrete actions
    approx.disc = any(strcmp(approx.type, {'rbfdisc', 'rbflindisc', 'triang', 'polydisc', 'regdisc'}));
end;

% type-specific fields
if strcmp(approx.type, 'regdisc'),
    if ~isfield(approx, 'singletree'),  approx.singletree = 0; end;
end;
