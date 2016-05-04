function a = sortx(a, th, lb, ub)
% Extended sort
%   A = SORTX(A, TH, LB, UB)
% Sort a vector removing (fuzzy) duplicates and bounding elements
%
% Parameters:
%   A       - the array to sort
%   TH      - (optional) threshold for different elements. If the distance between two consecutive
%           elements is at most TH, only the smaller one will be kept. Default 0.
%   LB      - (optional) the lower bound for the elements
%   UB      - (optional) the upper bound for the elements
% Returns:
%   A   - the sorted array, with duplicates removed, and optionally 
%       bounded elements


if nargin < 2, 
    th = 0;
end;
if nargin < 3,
    lb = -Inf;
    ub = Inf;
end;

a(a < lb) = lb; a(a > ub) = ub;   % bound
a = sort(a); 
a = [a(diff(a) > th) a(end)];     % remove (fuzzy) duplicates
