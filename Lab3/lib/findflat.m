function i = findflat(x, X, varargin)
% Finds a column vector in a flat matrix (row of column vectors)
%   I = FINDFLAT(x, X, VARARGIN)

f = all(repmat(x, 1, size(X, 2)) == X, 1);
if nargin == 2, i = find(f);
else            i = find(f, varargin{:});
end;
