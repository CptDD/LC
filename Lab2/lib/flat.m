function X = flat(x)
% Gives an explicit enumeration of the set of points in the cross product of n grids
%   X = FLAT(x)
% Parameters:
%   x       - cell array of grids
% Returns:
%   X       - flat representation of the grid elements. Each grid element is explicitly stored
%       on a column of X. X has dimension n x N where n is the number of dimensions and N the
%       total number of points on the grids.

% quick return when there is nothing to flatten
if length(x) == 1, X = x{1}; 
else
    n = length(x);
    xnd = ndgridx(x);
    X = zeros(n, numel(xnd{1}));
    for i = 1:n,
        X(i, :) = xnd{i}(:);
    end;
end;

