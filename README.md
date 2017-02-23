# GPR_DOB_MAP

Maximum a posteriori (MAP) state estimator for nonlinear dynamic systems using Gaussian process regression (GPR) model and derivative observations (DOB, also known as local linear models). This is the source code accompanying my CGNCC 2016 paper.

X. Yang, B. Peng, H. Zhou, and L. Yang, [State Estimation for Nonlinear Dynamic Systems using Gaussian Processes and Pre-computed Local Linear Models] (http://ieeexplore.ieee.org/document/7829090/ "IEEE Xplore"), 2016 IEEE Chinese Guidance, Navigation and Control Conference (CGNCC), Nanjing, 2016, pp. 1963-1968.

## Directory Structure
* GPR_DOB contains the code for gaussian process regression model with derivative observations. This is an exact replicate of the repository [gpr_dob](https://github.com/teancake/gpr_dob) with an appended 'sq_dist.c' and its compiled binary files to improve the execution speed of the original mfile 'sq_dist.m'
* GPR_DOB_MAP contains the main code for this repository, i.e. the MAP estimator using GPR model and derivative observations.
## Aircraft Angle of Attack Estimation Examples
There are two examples in this repository, both runs in MATLAB and estimates the angle of attack of an aircraft using measurements of other longitudinal vairables. This is a case study investigate in my CGNCC 2016 paper. The first example uses offline data collected from the aircraft simulation model, thus can be executed without Simulink. The other one uses Simulink and provices a more complete simulation environment.
### Example using offline data without Simulink 
This example includes a single m-file 'example_gpr_dob_map.m' and three data files: 'example_offline_data.mat' for the offline aircraft input output data,  	'example_GTM_IODATA.mat' and 'example_GTM_LINEAR_LON.mat' for the aircraft linearisation data used in MAP design. 

To run this example, simply run 'example_gpr_dob_map.m' in MATLAB.

### Example using Simulink
This example is a bit complicated. The two data files 'example_GTM_IODATA.mat' and 'example_GTM_LINEAR_LON.mat' used in MAP design are also used in this example. Apart from this, a simulink model 'example_cgncc16_paper_gtm.slx' is also included. Note that this model should be used in combination with the GTM simulation model. Befor running this example, the GTM Simulation model at https://github.com/nasa/GTM_DesignSim should be downloaded, a new Simulink model with the name of 'gtm_design_map.slx' should be created afterwards by cobmining the blocks in 'example_cgncc16_paper_gtm.slx' and 'gtm_sim.slx' in the GTM_DesignSim programs. The following figure shows how the two should be combined.
<img src="https://github.com/teancake/gpr_dob_map/blob/master/example_cgncc16_paper_simulink_blocks.png" alt="top" width="300px"> 

After this, run 'example_cgncc16_paper' in MATLAB. 

This example also uses an m-file 'ekfd.m', which implements an extended Kalman filter, the only reason for this is to re-use existing code, users can implement their own Kalman filter without bothering with the EKF.
