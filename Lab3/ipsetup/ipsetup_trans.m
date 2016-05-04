function dx = ipsetup_trans(t, x, m, u)
%  Implements the transition function of the inverted pendulum setup
%
%  Parameters:
%   T           - the current time. Ignored, added for compatibility with
%               the ODEXX function style.
%   X           - the current state
%   M           - the model structure specifying physical params
%   U           - the command
%
%  Returns:
%   DX          - the derivative of the state
%

dx(1,1) = x(2);
dx(2,1) = (m.mgl*sin(x(1)) - (m.b+m.K^2/m.R)*x(2) + m.Km*u) / m.J;

% end IPSETUP_TRANS returning DX
