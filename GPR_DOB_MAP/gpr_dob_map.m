function sys_xm_e = gpr_dob_map(sys_y, sys_u, sys_xm_e_prev)
% gpr_dob_map - Maximum a posteriori state estimator for nonlinear dynamic
% systems using a GPR model with derivative observations.
%
% input:
%   sys_y   is the current output of the system.
%   sys_u   is the input of the system with one-step time delay.
%   sys_xm_e_prev    is the state estimate at previous time instant.
%
% output:
%   sys_xm_e    is the current MAP estimate of the system's state.
%
% This program is not neatly implemented by using tons of global variables,
% these variables may be eliminated in further upgrades. Meaning of the
% global variables are below the declarations.
%
% Xiaoke Yang <das.xiaoke@hotmail.com> 
% Last modified: Tue 21 Feb 19:24:04 CST 2017

global X Y XD DY d_idx output_idx loghyp sys_x_lb sys_x_ub
% X - GP input data
% Y - GP output data (function output)
% XD - GP derivative observation input data
% DY - GP derivative observation output data
% d_idx - dimensions along which the derivatives are available
% output_idx - index of the system's output in the state
% loghyp - log hyper-parameters of the GP
% sys_x_lb - lower bound of the system's state estimate (mean value)
% sys_x_ub - upper bound of the system's state estimate (mean value)

disp('Estimating the state...')
% objective function and its derivative
fcn = @(v)gpr_dob_map_obj(v, sys_y, sys_u, sys_xm_e_prev, loghyp, ...
                                    	X, Y, XD, DY, d_idx, output_idx);
% d = checkgrad('gpr_dob_map_obj', xopt , 1e-6)
tmpv0 = sys_xm_e_prev;
tmpv0(output_idx) = sys_y;      % formulate the starting point
opts.v0 = tmpv0;                % starting point, the optimisation 
                                % is very sensitive to the starting point.
opts.maxIts = 200;              % maximum iterations
options = optimset('GradObj','on','Algorithm','sqp', ...
    'Display','iter','MaxIter',opts.maxIts,'TolFun',1e-5);
tic
sys_xm_e = fmincon(fcn, opts.v0,[],[],[],[],sys_x_lb,sys_x_ub,[],options);
toc


