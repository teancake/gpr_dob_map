function [xh, P] = ekfd(y, fx, hx, F, H, Q, R, Pd)
% ekfd is a Simulink/Embedded Matlab code compatible one-step discrete 
% extended Kalman estimator for a discrete-time plant.
%
%   [XH,P] = EKFD(Y,FX,HX,F,H,Q,R,) computes a one-step Kalman estimation
%   of the discrete-time plant
%
%        x_k = f(x_{k-1},u_{k-1}) + w_{k-1}      {State equation}
%        y_k = h(x_{k}) +  v_k                   {Measurements}
%
%   with process and measurement noise
%
%     E[w_k] = E[v_k] = 0,  Cov[w,w] = Q_k,  Cov[v,v] = R_k,  Cov[w,v] = 0.
%
%   Inputs: y - current measurement
%           fx - evaluation of f(x_{k-1},u_{k-1})
%           hx - evaluation of h(fx)  <== PAY ATTENTION TO THIS
%           F - evaluation of the linearisation of f(.)  at x_{k-1},u_{k-1}
%           H - evaluation of the linearisation of h(.) at fx  <== CAUTION
%           Q - covariance of w at k-1
%           R - covariance of v at k-1
%           Pd - P from the last time step
%   Outputs:xh - mean of the estimation of x_k
%           P - covariance of the estimation of x_k
%
%   Note: ekfd only returns the one step estimation of x_k ~ N(xh, P)
%   to run it recursively, simply feed P back to Pd.
%   Reference: http://en.wikipedia.org/wiki/Extended_Kalman_filter
%
%   Author: X. Yang (Control Lab, Cambridge University)
%   $Revision: 1.0 $
% $Last modified: 02-05-2013 16:52:08 X. Yang $

%#codegen

% predict
xp = fx;                    % Predicted state estimate
Pp = F * Pd * F' + Q;       % Predicted covariance estimate

% update
e = y - hx;                 % Innovation or measurement residual
S = H * Pp * H' + R;        % Innovation covariance
I = eye(size(S,1));
L = chol(S)';
invS = L'\(L\I);            % A better way of doing S^{-1} when S > 0
K = Pp * H' * invS;         % Near-optimal Kalman gain
xh = xp + K*e;              % Updated state estimate
I2 = eye(size(Pp,1));
P = (I2 - K * H) * Pp;      % Updated estimate covariance
