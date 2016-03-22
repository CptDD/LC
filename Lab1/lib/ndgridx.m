function X = ndgridx(x)
% Equivalent of NDGRID for which input and output are cell arrays
% instead of lists of arguments. Useful for working with variable-dimension grids.
%   X = NDGRIDX(x)
% Parameters:
%   x       - cell array of single-dimensional generating grids.
%   X       - cell array of resulting matrices
% See ndgrid().
% 
% ACKNOWLEDGMENT: This file uses portions of code from the file ndgrid.m, developed by MathWorks.


n = length(x);
for i=length(x):-1:1,
  x{i} = full(x{i}); % Make sure everything is full
  siz(i) = numel(x{i});
end
if length(siz)<n, siz = [siz ones(1,n-length(siz))]; end

X = cell(1,n);
for i=1:n,
  xi = x{i}(:); % Extract and reshape as a vector.
  s = siz; s(i) = []; % Remove i-th dimension
  xi = reshape(xi(:,ones(1,prod(s))),[length(xi) s]); % Expand
  X{i} = permute(xi,[2:i 1 i+1:n]); % Permute to i'th dimension
end
