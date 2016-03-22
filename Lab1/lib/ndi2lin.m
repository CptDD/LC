function lin = ndi2lin(ndi,dims)
%NDI2LIN Convert a decimal number to a "number" in an arbitrary base
%       per digit. Useful for converting indices of N-D arrays into
%       indices of linear arrays (reverse operation to LIN2NDI).
%       NDI2LIN(NDI,BASE) converts the number NDI expressed in the 
%       bases BASE into a decimal number. 
%
%       See also LIN2NDI.

% Copyright (c) Robert Babuska, 1998.

c = [1 cumprod(dims(1:length(dims)-1))];    % vector of "bases"
lin = (c*(ndi-1)')' + 1;                    % compute lin. index
