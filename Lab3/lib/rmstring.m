function a = rmstring(a, varargin)
% Remove one or more strings from a cell array of strings
% Parameters:
%   a   - cell array of strings
%   r   - strings to remove

r = varargin; 
if isempty(r), error('At least one string has to be removed'); end;

keepind = 1:length(a);
for i = 1:length(r),
    keepind(strcmp(a, r{i})) = 0;
end;

a = {a{find(keepind)}};