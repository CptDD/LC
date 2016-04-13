function dest = copyfields(src, dest, fields, overwrite)
% Copy values of the fields list FIELDS from structure SRC into structure DEST
%   DEST = COPYFIELDS(SRC, DEST, FIELDS)

% by default copy all source fields
if nargin < 3 || isempty(fields),
    fields = fieldnames(src);
end;
% default overwrite dest fields
if nargin < 4 || isempty(overwrite),
    overwrite = 1;
end;

for i = 1:length(fields),
    if isfield(dest, fields{i}) && ~overwrite, continue; end;
    dest.(fields{i}) = src.(fields{i});
end;
