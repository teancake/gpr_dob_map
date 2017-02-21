function [f,df] = gpr_dob_map_obj(sys_x, sys_y, sys_u, sys_xm_e_prev, ...
                                    loghyp, x, y, xd, dy, idx, output_idx)
% gpr_dob_map_obj - objective function and its gradient of the MAP state 
% estimator for nonlinear dynamic systems using a GPR model with 
% derivative observations.
%
% input:
%   sys_x   is the state of the system, and is the optimisation variable.
%   sys_y   is the current output of the system.
%   sys_u   is the input of the system with one-step time delay.
%   sys_xm_e_prev    is the state estimate at previous time instant.
%   loghyp  is the log-hyperparameters of the GPR model.
%   x      	is a n by D matrix of training inputs
%   y     	is a (column) vector (of size n) of targets
%   xd		is a nd by D matrix of training inputs where derivatives are
%               observed.
%   dy	  	is a column vector of ndxD by 1 of derivatives at xd.
%   idx	  	is a subvector of [1:D], determining the dimensions of x 
%               along which derivative observations are available.
%   output_idx  is the index of the system's output in the state.
%
% output:
%   f       is the value of the obejctive function.
%   f       is the gradient of the objective function.
%
%
% Xiaoke Yang <das.xiaoke@hotmail.com> 
% Last modified: Tue 21 Feb 20:11:07 CST 2017

sys_x = sys_x(:);
xtmp =  [sys_x' sys_u];             % GP input formulated by the system 
                                    % state and input

% negative log likelihood
f = 0; df = 0; j=0;
for i = output_idx 
    j = j + 1;
    [m, S, dm, dS] = gpr_dob(loghyp(:,i),x,y(:,i), xd, dy(:,i), idx, xtmp);
    invS = 1/S;
    lik = 0.5* (sys_y(j)-m)'* invS * (sys_y(j)-m) + 0.5*log(S) ...
        + 0.5*log(2*pi);
    dlik = 0.5*(invS*dS-(sys_y(j)-m)^2*invS^2*dS - 2*(sys_y(j)-m)*invS*dm);
    f = f+ lik;
    df = df + dlik;
end

% now make predictions
N_sys_x = numel(sys_xm_e_prev);
sys_xm_p = zeros(N_sys_x,1);  	% predicted system state - mean
sys_xs_p = eye(N_sys_x);    	% rredicted system state - variance
for i = 1:size(y,2)
    [out1, out2] = gpr_dob(loghyp(:,i), x, y(:,i), xd, dy(:,i), idx, ...
                                [sys_xm_e_prev;sys_u]');
    sys_xm_p(i) = out1;
    sys_xs_p(i,i) = out2;
end

% negative log of prior distribution
dprior = sys_xs_p\(sys_x - sys_xm_p);       % derivative
prior = 0.5*(sys_x - sys_xm_p)'*dprior;     % negative log of prior
f = f + prior;                              % MAP objective function
df = df(1:numel(sys_x)) + dprior;           % objective function derivative