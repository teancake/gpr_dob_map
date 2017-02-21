function logtheta = gpr_dob_training(x, y, xd, dy, idx, logtheta_lb, ...
                                                            logtheta_ub)
% grp_dob_training - Training of a GPR model with derivative observations
% input:
%   x        is a n by D matrix of training inputs
%   y        is a (column) vector (of size n) of targets
%   xd	     is a nd by D matrix of training inputs where derivatives are
%               observed.
%   dy	     is a column vector of ndxD by 1 of derivatives at xd.
%   idx	     is a subvector of [1:D], determining the dimensions of x 
%               along which derivative observations are available.
%   logtheta_lb     is the lower bound of the log-hyperparameters.
%   logtheta_ub     is the upper bound of the log-hyperparameters.

%
% output:
%   K	    is the trained hyper-parameter
%
% This program is supposed to run with gpr_dob.m which implements Gaussian
% process regression with derivative observations. This program utilises
% the fmincon function in MATLAB, and has very preliminary implementations.
%
% Xiaoke Yang <das.xiaoke@hotmail.com> 
% Last modified: Tue 21 Feb 19:24:04 CST 2017

E = size(dy,2);
logtheta = logtheta_lb;
for i = 1:E
    fcn = @(v)gpr_dob(v(:), x, y(:,i), xd, dy(:,i), idx);
    opts.v0 = (logtheta_lb+logtheta_ub)/2;      % starting point
    opts.maxIts = 200;                          % maximum iterations
    disp(sprintf('Training GP #%d...',i))
    % options for fmincon
    options = optimset('GradObj','on','Algorithm','interior-point', ...
        'Display','iter','MaxIter',opts.maxIts,'TolFun',1e-5);
    % use fmincon
    vout = fmincon(fcn, opts.v0,[],[],[],[],logtheta_lb,logtheta_ub,...
                [],options);
    
    logtheta(:,i) = vout;
end