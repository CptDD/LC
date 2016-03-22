function info = getenvx(what, includever)

if nargin < 1, what = 'all'; end;
if nargin < 2, includever = 0; end;

% parse a character includever
if ischar(includever),
    switch lower(includever),
        case {'yes','y'},
            includever = 1;
        case {'no','n'}
            includever = 0;
        otherwise,
            includever = 0;
    end;
end;

switch what
    case 'all',
        info.computer = computer;
        info.matlab_version = version;
        
        if strfind(info.computer, 'WIN'),       % windows
            [code output] = system('set');
            if code == 0, info = parsevariables(output, info, '='); end;
        elseif strfind(info.computer, 'GLNX'),  % linux
            [code output] = system('cat /proc/cpuinfo');
            if code == 0, info.cpu_info = parsevariables(output, struct, ':'); end;
            [code output] = system('cat /proc/meminfo');
            if code == 0, info.mem_info = parsevariables(output, struct, ':'); end;
            [code output] = system('env');
            if code == 0, info = parsevariables(output, info, '='); end;
        end;
        
        if includever,
            info.ver = ver;
        end;
        
    case 'computer',
        info = computer;
    case 'version',
        info = version;
    case 'ver',
        info = ver;
        
    otherwise,
        info = getenv(what);
end;

end
% -------------------------------------------------

function p = parsevariables(s, p, assign, pref)

printable = 32:127;
eol = [10 13];
if nargin < 2,  p = struct;         end;
if nargin < 3,  assign = '=';       end;
if nargin < 4,  pref = '';          end;

while true,
    i = 1;
    while i <= length(s) && all(s(i) ~= printable), i = i+1; end;
    if i > length(s), break; end;
    [name s] = strtok(s(i:end), assign);
    [val s] = strtok(s(2:end), eol);
    name = strtrim(name); val = strtrim(val);
    p.([pref makegoodfieldname(name)]) = val;
end;

end
% -------------------------------------------------

function n2 = makegoodfieldname(n)

firstchar = ['a':'z' 'A':'Z'];
innerchar = ['a':'z' 'A':'Z' '_' '0':'9'];

start = 1;
while start <= length(n) && all(n(start) ~= firstchar), start = start + 1; end;

if start > length(n),
    n2 = 'defaultname';
else
    indices = [];
    for i = start:length(n),
        if any(n(i) == innerchar),
            indices(end+1) = i;
        end;
    end;
    n2 = n(indices);
    if isempty(n2), n2 = 'defaultname'; end;
end;

end
% -------------------------------------------------
