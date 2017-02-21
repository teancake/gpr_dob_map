%% README
% This file is an example to test the MAP state estimator for nonlinear
% systes using Gaussian process regression models and derivative
% observations (a.k.a, local linear models). 
%
% The test scenario in this file is the estimation of aircraft angle of
% atack from measurement of other longitudinal variables. The scenario is
% the same as the study case of the reference below, apart from this 
% example runs on offline data collected from the GTM simulation model. 
% For detailed information, please refer to the following reference.
%
% Xiaoke Yang, Bo Peng, Hang Zhou and Lingyu Yang, "State estimation for 
% nonlinear dynamic systems using Gaussian processes and pre-computed 
% local linear models," 2016 IEEE Chinese Guidance, Navigation and Control 
% Conference (CGNCC), Nanjing, 2016, pp. 1963-1968.
% 
% This program runs in MATLAB since it utilises the fmincon optimisation
% function in MATLAB. The program may be modified to run in Octave if other
% optimisation procedure is used. Since the nonlinear optimisation in this
% program is time consuming, the execution of this example may take a few
% minutes. By compiling GPR_DOB/sq_dist.c, the execution time may be
% reduced to less than a half.
%
% Xiaoke Yang <das.xiaoke@hotmail.com>
% Last modified: Tue 21 Feb 19:16:35 CST 2017
%
%% PATH CONFIG
addpath('GPR_DOB_MAP')          % include the MAP code
addpath('GPR_DOB')              % include the GPR_DOB code
%% CLEAR ALL WORKSPACE VARIABLES THEN RELOAD DATA
clear all
load example_offline_data	% load offline simulation data
load example_GTM_LINEAR_LON	% GTM longitudinal model
load example_GTM_IODATA   	% XT YT, data at the initial operating point

%% GLOBAL VARIABLES
global X Y XD DY d_idx loghyp output_idx sys_x_lb sys_x_ub
% X - GP input data
% Y - GP output data (function output)
% XD - GP derivative observation input data
% DY - GP derivative observation output data
% d_idx - dimensions along which the derivatives are available
% loghyp - log hyper-parameters of the GP
% output_idx - index of the system's output in the state
% sys_x_lb - lower bound of the system's state estimate (mean value)
% sys_x_ub - upper bound of the system's state estimate (mean value)

%% SYSTEM CONFIGURATIONS

NOISE_ENABLE = 0;                       % enable noise or not
XT = XT*diag([1/0.59 1 1 1 1]);         % Vtas knots --> feet/s
YT = YT*diag([1/0.59 1 1 1]);           % Vtas knots --> feet/s
model_idx = [6 33];% 15 33];            % indices for the local models used
GTM_linear = GTM_linear(model_idx);     % obtain the local models
x0 = x0(model_idx);                     % operating points

Ts = 0.02;                              % sampling period
Ts_data = mean(diff(data_t));           % sampling period of the data
dt = floor(Ts/Ts_data);                 % compute data interval
data_t = data_t(1:dt:end);              % resample the data
data_x = data_x(1:dt:end,:);	
data_u = data_u(1:dt:end,:); 

state_idx = [1 2 3 4];      % indices for the states, Vtas, alpha, q, theta
input_idx = [1];            % indices for the inputs, elevator
input_idx_u0 = [1];         % indices for the inputs of the operating points
rho_idx = 2;                % rho_idx =2 corresponding to alpha
output_idx = state_idx(state_idx~=rho_idx);  	% output index

Nu = numel(input_idx);      % number of system input: elevator
Nx = numel(state_idx);      % number of system state: V, alpha, q, theta
M = numel(x0);              % number of local linear models
D = Nx+Nu;                  % number of GP input
E = Nx;                     % number of GP output


Ad = zeros([size(GTM_linear{1}.A) M]);      % discretised system matrices
Bd = zeros([size(GTM_linear{1}.B) M]);      % discretised input matrices
F0d = zeros([size(GTM_linear{1}.A,1) M]);   % constant terms

sys_x_lb = [50/0.592484 0/180*pi -1 -5/180*pi ]'; % state lower bounds  
sys_x_ub = [90/0.592484 20/180*pi 1 30/180*pi]';  % state upper bounds 


%% DERIVATIVE AND FUNCTION OBSERVATION DATA
XD = [];                    % input data
DY = [];                    % derivative data
Y = [];                     % function output data
for i=1:M
    % discretise the system
    x0{i}(4) = x0{i}(4)/180*pi;                 % theta deg -> rad
    x0{i}(2) = x0{i}(2)/180*pi;                 % alpha deg -> rad
    x0{i}(1) = x0{i}(1)/0.592484;               % tas knots -> feet/s
    Ad(:,:,i) = GTM_linear{i}.A*Ts + eye(Nx);   % use 1st order approx.
    Bd(:,:,i) = GTM_linear{i}.B*Ts;             % to ZOH
    F0d(:,i) = x0{i};                           % F0d = F0*Ts + x0
    % formulate derivative data from local linear models
    XD = [XD; [x0{i}(state_idx) u0{i}(input_idx_u0)]];
    dy = [Ad(state_idx,state_idx,i) Bd(state_idx,input_idx,i)]';
    DY = [DY; dy];
    Y =  [Y; F0d(state_idx,i)'];
end
X = XD;

if NOISE_ENABLE         % if use noise, we'll need some offline data
    MT = 10;            % in order to infer the noise variance
else
    MT = 0;
end

X = [X; XT(101:100+MT,[state_idx end])];    % input-output data
Y = [Y; YT(101:100+MT,state_idx)];
d_idx = 1:D;                % dimensions along which we have derivatives

%% GP CONFIGURATIONS
% hyper parameters
% format of hyp-params
%   hyp = log[ell11 ... ell1E; ... ;ellD1 ellDE; sf1 ... sfE; sn1 ... snE]
%
ell = ones(D,E);                                % characteristic length
sf = ones(1,E);                                 % size of the function
sn = 0.01*ones(1,E);                            % size of the noise
loghyp_lb  = log([ones(D+1,1)*1e-2; 1e-4]);     % lower bound
loghyp_ub  = log([ones(D+1,1)*1e5; 1e4]);       % upper bound
% train the GP
loghyp = gpr_dob_training(X, Y, XD, DY, d_idx, loghyp_lb, loghyp_ub)

%% KALMAN FILTER CONFIGURATIONS
% system matrices
sys.A = Ad(state_idx,state_idx,1);
sys.B = Bd(state_idx,1,1);
sys.C = [1 0 0 0; 0 0 1 0; 0 0 0 1];
sys.D = zeros(numel(state_idx),numel(input_idx));
% operating points at which A B C D are obtained
sys.x0 = x0{1}';
sys.u0 = u0{1}';
sys.u0 = sys.u0(input_idx);
% P0 Q R
kf.P0 = eye(size(sys.A));
kf.Q = eye(size(sys.A))*0.1;
kf.R = diag([100 10 1]);
% Kalman gain
[~, ~, kf.L] = dare(sys.A',sys.C',kf.Q,kf.R);


%%  SIMULATION
sys_xm_e_kf_delta = zeros(size(sys.x0));    % initial state estimate - KF
sys_xm_e_gp = zeros(Nx,1);                  % initial state estimate - GP
sys_xm_e_kf_v = [];
sys_xm_e_gp_v = [];
G_pre = diag([1/0.592484 1 1]);         % the unit conversion is due to
G_post = diag([0.592484 180/pi 1 1]);   % different units used in the 
figure(1)                               % linearisation of the system
clf
for i=1:numel(data_t)
    if NOISE_ENABLE
        sys_y_n = data_x(i,output_idx)' + diag([1 0.032 0.032])*randn(3,1);
    else
        sys_y_n = data_x(i,output_idx)';
    end
        
    sys_y = G_pre*sys_y_n;        % unit conversion
    % There is one-step delay on the u both for the KF and GPR-DOB-MAP
    if i == 1
        sys_u = zeros(1,Nu);
        sys_u_delta = zeros(1,Nu);
    else
        sys_u = data_u(i-1,:);
        sys_u_delta = sys_u - sys.u0;           % Kalman filter uses  
    end                                         % perturbed input and state 
    sys_y_delta = sys_y - sys.x0(output_idx);   % w.r.t. the equilibrium.
    sys_xm_e_kf_delta = sys.A*sys_xm_e_kf_delta + sys.B*sys_u_delta ...
        + kf.L'*(sys_y_delta - sys.C*sys_xm_e_kf_delta);  % Kalman filter
    sys_xm_e_kf = sys_xm_e_kf_delta + sys.x0;   % append the equil. state
    sys_xm_e_gp = gpr_dob_map(sys_y, sys_u, sys_xm_e_gp);    % GPR-DOB-MAP
    sys_xm_e_kf_v = [sys_xm_e_kf_v; (G_post*sys_xm_e_kf)']; % save results
    sys_xm_e_gp_v = [sys_xm_e_gp_v; (G_post*sys_xm_e_gp)']; % and unit conv
    % dynamic plotting
    subplot(4,1,1)
    title(sprintf('t=%f s, finished %2.2f%%',data_t(i), ...
                                    data_t(i)/data_t(end)*100));
    plot(data_t(1:i),data_x(1:i,1),'g-', data_t(1:i), ...
        sys_xm_e_kf_v(:,1),'r-.', data_t(1:i),sys_xm_e_gp_v(:,1),'b--')
    legend('True value','Kalman filter','GPR-DOB-MAP')
    ylabel('V_{TAS}[knots]')
    xlim([0 data_t(end)])
    subplot(4,1,2)
    plot(data_t(1:i),data_x(1:i,2),'g-',data_t(1:i), ...
        sys_xm_e_kf_v(:,2),'r-.', data_t(1:i),sys_xm_e_gp_v(:,2),'b--')
    legend('True value','Kalman filter','GPR-DOB-MAP')
    ylabel('AoA[deg]')
    xlim([0 data_t(end)])
    subplot(4,1,3)
    plot(data_t(1:i),data_x(1:i,3),'g-',data_t(1:i), ...
        sys_xm_e_kf_v(:,3),'r-.', data_t(1:i),sys_xm_e_gp_v(:,3),'b--')
    legend('True value','Kalman filter','GPR-DOB-MAP')
    ylabel('q[rad/s]')
    xlim([0 data_t(end)])
    subplot(4,1,4)
    plot(data_t(1:i),data_x(1:i,4),'g-',data_t(1:i), ...
        sys_xm_e_kf_v(:,4),'r-.', data_t(1:i),sys_xm_e_gp_v(:,4),'b--')
    legend('True value','Kalman filter','GPR-DOB-MAP')
    ylabel('\theta[rad]')
    xlim([0 data_t(end)])
    drawnow
end

