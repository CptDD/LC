function cfg = str2cfg(str, keywords)
% String configuration to structure
%   CFG = STR2CFG(STR, KEYWORDS)
% Parameters:
%   str         - the configuration string
%   keywords    - keywords to search in the string, will become structure fields
%       Should be a cell array of strings, or a structure. In the second case the structure
%       field names will be used as keywords.
% Returns:
%   cfg         - string configuration
% 
% Binary, numeric or string values are supported. Fields which are present
% on the string, but for which no value is assigned are treated as boolean
% "true" values.
% E.g.
%   cfg = str2cfg('alpha beta=2.5 gamma=value', {'alpha', 'beta', 'gamma', 'delta'}) produces
% cfg = alpha: 1        (boolean true)
%        beta: 2.500
%       gamma: 'value'
%       delta: 0        (boolean false)
% If str is already a structure, the same structure is returned.

if isstruct(str),           % assume already given as structure
    cfg = str;
    return;
end;
if isstruct(keywords),      % obtain field names
    keywords = fieldnames(keywords); 
end;

slen = length(str);

cfg = struct;
for i = 1:length(keywords),
    kw = keywords{i}; kwlen = length(kw);
    indices = strfind(str, keywords{i});
    if isempty(indices), continue; end;      % no setting found
    accept = 0;
    for j = 1:length(indices),
        ix = indices(j); after = ix + kwlen; before = ix - 1;
        % accept only if entire keyword (i.e., preceded and followed by separator, 
        % or beginning / ending the string
        if ((after > slen) || (str(after) == '=') || (str(after) == ' ')) ...
                && ((before < 1) || (str(before) == ' ')),
            accept = 1;
            break;
        end;
    end;
    if ~accept, continue; end;
    % if no value given, interpreted as boolean true
    if (after > slen) || (str(after) == ' '),
        cfg.(kw) = true;
        continue;
    end;
    % otherwise, parse after equal
    val = strtok(str(after+1:end), ' ');
    % convert to various data types if possible
    % if it starts with a letter, it's a string
    if ~any(val(1) == ['a':'z' 'A':'Z']) && ~isempty(str2num(val)),      % try numeric
        val = str2num(val);
    elseif val(1) == '{',           % cell array
        try val = eval(val);
        catch   % no change, leave it as string
        end;
    end;
    cfg.(kw) = val;
end;
