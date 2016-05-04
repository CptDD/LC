function [ind, mu, indp] = mdegs_p(x, c, roll, n, p, tab)
%  Computes the membership degrees in a P-dimensional conjunction of triangular membership functions.
%   [IND, MU, INDP] = MDEGS_P(X, C, ROLL, N, P, TAB)
%
%  Computes membership degrees for a P-dimensional variable in a p-dimensional conjunction of
%  triangular membership functions. Each P-dimensional fuzzy set is a conjunction (with the
%  product rule) of triangular single-dimensional (1D) fuzzy sets. These 1D sets are normal and
%  the support of each fuzzy set is the interval defined by the centers of the two adjacent
%  fuzzy sets, such that the sum of membership degrees is always one. Each such collection of 1D
%  fuzzy sets is completely determined by the array of centers.
%
%  The computation of the membership degree is optimized, such that the complexity is O(P) where
%  P is the number of dimensions, and the built-in Matlab operations are not considered.
%
%  Parameters:
%   X           - the variable value, a P-dimensional vector (column or row)
%   C           - the centers of the fuzzy sets. A P-element cell array, with each element
%       C{i} a row vector containing the triangle centers across dimension i.
%       The elements of the vector must be unique and sorted in ascending order.
%   ROLL        - rollover flags for the membership functions. Currently not used, reserved.
%   N           - the numbers of fuzzy sets, N{i} = length(C{i})
%   P           - the dimension of the variable, P = length(X) = length(C) = length(N)
%   TAB         - helper variable, should contain DEC2BASE(0:2^p-1, 2) - 47
%   
%  Returns:
%   IND         - the linear indices of the activated p-dimensional membership degrees. A column
%       vector of length 2^P. The activated membership degrees are usually the nonzero membership
%       degrees. For ease of use and implementation, however, some zero membership degrees will
%       be regarded as activated to keep the length of ind constant.
%   MU          - the column vector of activated membership degrees (some of these may be zero). The
%       membership degrees in MU correspond 1-to-1 with the indices in IND.
%   INDP        - the P-dimensional indices of the activated p-dimensional membership degrees.
%       A matrix of dimension 2^P x P, with one P-dimensional index on each row. The rows of
%       INDP correspond 1-to-1 with the membership degrees in MU and the linear indices in IND.

% Author:   Lucian Busoniu
% Version:  1.0
% Date:     2007-06-28

indp = zeros(bitshift(1, p), p);     % zeros(2^p, p)
mu = indp;

% compute 1-dimensional mdegs and indices of activated 1-dimensional MFs
for ip = p:-1:1,
    % find i such that x \in [c(i), c(i+1)); i=n if x = last center value
    i = find(c{ip} <= x(ip), 1, 'last');
    if i < n(ip),           % i, i+1 MFs are activated;
        m  = (c{ip}(i+1) - x(ip)) / (c{ip}(i+1) - c{ip}(i));
        mv = [m;  1-m];         % activation degrees
        iv = [i;  i+1];         % and indices
    else                    % singleton on the last MF
        mv = [  0;  1];         % activation degrees (first is a dummy)
        iv = [i-1;  i];         % and indices (first is a dummy)
    end;
    
    % place on expanded index and membership table for later multiplication
    % tab = dec2base(0:(2^p-1), 2) - 47; 47 is '0'-1 s.t. elements are 1-based
    indp(:, ip) = iv(tab(:,  ip));
    mu(:, ip)  = mv(tab(:,  ip));
end;

mu = prod(mu, 2);           % membership degrees of the activated p-dimensional MFs
ind = ndi2lin(indp, n);     % their linear indices

% END mdegs_n RETURNING ind, mu, indp ---------------------------

