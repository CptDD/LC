function ndi = lin2ndi(lin,dims)
%LIN2NDI Useful for generating indices of N-D arrays.
%		LIN2NDI(N,BASE) returns the representation of N as a vector
%		in the bases given by the elements of BASE. (Reverse operation
%		to NDI2LIN.)
%
%		Example:
%			A = zeros(2,2,3,4); 
%			A(1,2,3,4) = 1;
%			ndi = lin2ndi(find(A),size(A))
%
%		See also NDI2LIN.

% Copyright (c) Robert Babuska, 1998.

lin = lin(:); N = length(lin); 						% get dimensions
nl = length(dims);									% get dimensions
c = [1 cumprod(dims(1:nl-1))];						% vector of divisors
ndi = floor((lin-1)*ones(1,nl)./(ones(N,1)*c));	    % entier
ndi = 1 + rem(ndi,ones(N,1)*dims);					% reminder
