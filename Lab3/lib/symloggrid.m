function gr = symloggrid(n, b, sp, ep)
% Create symmetrical logarithmic grid. Supports one or more variables
% Parameters:
%   n   - #(s) of points
%   b   - bound(s) of variable(s) (just the positive one, assumed symmetrical around origin)
%       must have same length as n
%   sp, ep  - start and end power of grid, used to control the spacing

nn = length(n); nb = length(b);
if nb > 1 && nn == 1,
    n = n + zeros(nb, 1);
elseif nb ~= nn,
    error('n and b must have the same length, or n must be scalar');
end;

if nargin < 3, sp = 0; end;
if nargin < 4, ep = 1; end;

if any(mod(n, 2) == 0),
    error('A symmetrical log grid can only have an odd number of elements');
end;

if nb == 1,
    ls = (logspace(sp, ep, ceil(n/2)) - 10^sp)/(10^ep - 10^sp);
    gr = sortx([-b*ls b*ls]);
else
    gr = cell(nb, 1);
    for i = 1:nb,
        ls = (logspace(sp, ep, ceil(n(i)/2)) - 10^sp)/(10^ep - 10^sp);
        gr{i} = sortx([-b(i)*ls b(i)*ls]);
    end;
end;

end
