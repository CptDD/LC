%% Near-optimal solution inspection
% Note this was also computed with fuzzy QI, with a very fine approximator; because fuzzy
% Q-iteration is consistent, this guarantees a close approximation of the optimal solution.
% we use the same syntax to extract the approximator
optcfg = struct;
optcfg.approx = 1;
optcfg.datafile = 'ip_nearopt';
[optapprox, opttheta] = fuzzyqi(optcfg);

figure; set(gcf, 'position', [0, 0, 800,400]); movegui(gcf, 'center');
% we are plotting on a finer grid (npoints) than above, to take advantage of the precision in the
% solution
%subplot(121); optapprox.plotv(optapprox, opttheta, 'npoints=60'); xlabel('alpha'); ylabel('alpha'''); zlabel('max_u Q*(x, u)');
%subplot(122); optapprox.ploth(optapprox, opttheta, 'npoints=60'); xlabel('alpha'); ylabel('alpha'''); title('h*(x)');
