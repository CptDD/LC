function d = dispx(message, verb, level, varname, vartype)
% Message display with extended functionality
%   D = DISPX(MESSAGE, VERB, LEVEL)
% Displays a message conditioned on its priority.
%
% Parameters
%   MESSAGE     - message to display
%   VERB        - verbosity level of current context
%   LEVEL       - priority level of the message (the lower the number, the higher the
%               priority)
%   VARNAME
%   VARTYPE
%
% Returns:
%   D           - 1 if string displayed, 0 otherwise

if nargin < 2,  verb = Inf; end;        % show everything
if nargin < 3,  level = 0; end;
if level > verb, d = 0; return; end;    % fast return
if nargin < 4,  varname = 'msg'; end;
if nargin < 5,  vartype = []; end;

if level <= verb, 
    if strcmp(vartype, 'char') || ischar(message), disp(message); 
    else    % display the message as a normal variable, using any given varname
        % process special type of variables
        if strcmp(vartype, 'config'),
            % configuration structure -- remove all empty fields
            f = fieldnames(message);
            efs = ''; efc = {};   % empty fields
            for i = 1:length(f),
                fv = message.(f{i});
                if isempty(fv) || (isnumeric(fv) && length(fv) == 1 && fv == 0),
                    efc{end+1} = f{i}; efs = [efs f{i} ', '];
                end;
            end;
            if ~isempty(efc),
                efs = efs(1:end-2); % remove the last useless comma and space
                % remove empty fields
                message = rmfield(message, efc);
            end;
            eval([varname '=message']);
            if ~isempty(efc), disp(['Empty or 0 ' varname ' fields: ' efs]); end;
        else
            eval([varname '=message']);
        end;
    end;
    d = 1;
else d = 0; 
end;

% END dispx() RETURNING d =============================================================