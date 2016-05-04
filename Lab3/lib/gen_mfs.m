% Convenient generation of triangular MFS
function mfs = gen_mfs(c, rollover, rolloverdist)

if nargin < 2, rollover = 0; rolloverdist = 0;
elseif nargin < 3 && rollover > 0, 
    rolloverdist = c(2) - c(1);      % assumed equidistant
end;

mfs.c = c';
mfs.type = 't';
mfs.n = length(c);
mfs.rollover = rollover;
mfs.rolloverdist = rolloverdist;

