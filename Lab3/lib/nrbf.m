function a = nrbf(x, N, c, rad)
% Computes normalized radial basis functions values for a continuous x
%   A = NRBF(X, N, C, RAD)
%
% Parameters:
%   X       - continuous variable (column vector of dimension p)
%   N       - number of basis functions
%   C       - centers of basis functions, one on each column, size p x N
%   RAD     - radii of basis functions, one on each column, size p x N
% Returns:
%   A       - normalized RBF values for given X, 1xN

a = exp(-sum( ((repmat(x, 1, N)  - c) ./ rad).^2, 1 ));
a = a/sum(a);

end
% END nrbf returning a ------------------------------------------------------
