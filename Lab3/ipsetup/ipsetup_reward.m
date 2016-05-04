function rplus = ipsetup_reward(m, x, u, xplus)
% Computes rewards for the IP setup

% Reward
if m.rewtype(1) == 'l',         % 'lqr'
    rplus = - x' * m.Qrew * x - m.Rrew * u * u;
elseif m.rewtype(1) == 'n',     % 'nlqr'
    rplus = 1 - (x' * m.Qrew * x + m.Rrew * u * u) / m.nonnorm_maxr;
elseif strcmp(m.rewtype, 'refnlqr'),
    rplus = 1 - ((x-m.xref)' * m.Qrew * (x-m.xref) + m.Rrew * u * u) / m.nonnorm_maxr;
elseif m.rewtype(1) == 'b',     % box
    rplus = all(abs(x) <= m.boxrew);
elseif m.rewtype([1 2]) == 'sh',% shaping + lqr
    rplus = - x' * m.Qrew * x - m.Rrew * u * u + m.gamma * ips_shaping(m, xplus) - ips_shaping(m, x);
elseif m.rewtype(1) == 'g',     % gauss
    rplus = -1 + exp(-[x; u]' * m.W * [x; u]);
elseif m.rewtype([1 2]) == 'si',  % sine
    rplus = m.c1 * cos(xplus(1)) - m.c2 * abs(xplus(2));
end;

% uncomment to enable scaling
rplus = (1-m.scaling) + rplus * m.scaling;

end
% END ipsetup_reward

% Local shaping function
function s = ips_shaping(m, x)
% if m.rewtype(end) == 'x',   % 'shapbox'
    % Box ("band") shaping function
    s = m.cshap * all(abs(x) < m.bandshap);
% end;
end
