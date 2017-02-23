%% README
% This file is an example to test the MAP state estimator for nonlinear
% systes using Gaussian process regression models and derivative
% observations (a.k.a, local linear models). 
%
% This example provides the full simulation scenario in the the following 
% reference, and should run together with the GTM simulation model at
% https://github.com/nasa/GTM_DesignSim
%
% Xiaoke Yang, Bo Peng, Hang Zhou and Lingyu Yang, "State estimation for 
% nonlinear dynamic systems using Gaussian processes and pre-computed 
% local linear models," 2016 IEEE Chinese Guidance, Navigation and Control 
% Conference (CGNCC), Nanjing, 2016, pp. 1963-1968.
% 
% Since the nonlinear optimisation in this program is time consuming, 
% the execution of this example may take a few minutes. By compiling 
% GPR_DOB/sq_dist.c, the execution time may be reduced to less than a half.
%
% This example also uses an m-file ekfd.m, which implements an extended
% Kalman filter, the only reason for this is to re-use existing code, users
% can implement their own Kalman filter without bothering with the EKF.
%
% Befor running this example, the GTM Simulation model at 
% https://github.com/nasa/GTM_DesignSim should be downloaded, a new
% Simulink model with the name of 'gtm_design_map.slx' should be created
% by cobmining the blocks in 'example_cgncc16_paper_gtm.slx' and 
% 'gtm_sim.slx' in the GTM_DesignSim programs. 
%
% Xiaoke Yang <das.xiaoke@hotmail.com>
% Last modified: Thu 23 Feb 11:18:51 CST 2017
%
%% PATH CONFIG
addpath('GPR_DOB_MAP')          % include the MAP code
addpath('GPR_DOB')              % include the GPR_DOB code
% GTM Paths
addpath(genpath('./config/'));  % Add mat file directory
addpath('./mfiles');            % Add mfiles directories.
addpath('./obj');               % Add compilex code directory
addpath('./libs');              % Add libriary directory
rehash path
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



%%  SIMULATION
sys_xm_e_kf_delta = zeros(size(sys.x0));    % initial state estimate - KF
sys_xm_e_gp = zeros(Nx,1);                  % initial state estimate - GP
G_pre = diag([1/0.592484 1 1]);         % the unit conversion is due to
G_post = diag([0.592484 180/pi 1 1]);   % different units used in the 
                                        % linearisation of the system

%% start simulation
open_system('gtm_design');
% load nominal starting point
loadmws(init_design('GTM'),'gtm_design');
% Trim model, and load trimmed conditions
SimVars = trimgtm(struct('eas',75, 'gamma',0));

hWS = get_param('gtm_design', 'ModelWorkspace');
hWS.reload;
close_system('gtm_design');

open_system('gtm_design_map')
% Load Simulation Variables (at trim condition) into Model Workspace
loadmws(SimVars);

% Construct same doublet sequence via simulink
set_param('gtm_design_map/Input Generator/Doublet Generator', ...
                                        	'timeon','[0.5 12 7]');
set_param('gtm_design_map/Input Generator/Doublet Generator', ...
                                            'pulsewidth','[0.5 2 2]');
set_param('gtm_design_map/Input Generator/Doublet Generator', ...
                                            'amplitude','[5 0 0]');
% start simulation
set_param('gtm_design_map', 'SimulationCommand', 'start')
