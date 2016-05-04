function approx = sample_approx(model, cfg)
% A generic (usually parametric) function approximator
% This is an example file explaining the interface exposed by the approximators in the ApproxRL
% toolbox.
% The approximator is created by calling sample_approx(model, cfg) where model is a problem model
% and the config fields are specific to each approximator. The result is a structure (call it
% "approx" for the sake of example) which generally provides the following standard fields:
%   disc            - boolean, whether the approximator is discrete-action or not
%   sparse          - boolean, whether the approximator works with sparse basis function vectors
%   extendedsyntax  - boolean, whether the approximator supports extended syntax, see below
%   phi             - a function for computing the BF vector for a given state-action pair. 
%       Signature: phi = approx.phi(approx, x, u)
%   q               - a function for computing the approximate Q-value of a state-action pair.
%       Signature: q = approx.q(approx, theta, x, u)
%       If the approximator supports extended syntax, can also return the BF vector phi:
%                  [q, phi] = approx.q(approx, theta, x, u)
%   h               - a function for finding a policy action for a state. The policy is greedy,
%       i.e., it will always be the case that max_u Q(x, u) = Q(x, h(x))
%       Signature: h = approx.h(approx, theta, x)
%       If the approximator supports extended syntax, can also return the maximal Q-value:
%                  [u, qstar] = approx.h(approx, theta, x)
%   plotv           - a function for plotting a representation of the function V(x) = max_u Q(x, u)
%       Signature: approx.plotv(t, theta, arg1, arg2, ...)
%       See also approx_plotv
%   ploth           - a function for plotting a representation of the greedy policy
%       Signature: approx.ploth(t, theta, arg1, arg2, ...)
%       See also approx_ploth
%
%   The arguments these functions use are:
%   approx          - the approximator structure
%   theta           - the parameters (usually a vector)
%   x               - the state
%   u               - the action




% do nothing, just a placeholder function
approx = struct;