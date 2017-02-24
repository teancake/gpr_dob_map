# GPR_DOB_MAP

Maximum a posteriori (MAP) state estimator for nonlinear dynamic systems using Gaussian process regression (GPR) model and derivative observations (also known as local linear models). This is the source code accompanying my CGNCC 2016 paper.

X. Yang, B. Peng, H. Zhou, and L. Yang, [State Estimation for Nonlinear Dynamic Systems using Gaussian Processes and Pre-computed Local Linear Models] (http://ieeexplore.ieee.org/document/7829090/ "IEEE Xplore"), 2016 IEEE Chinese Guidance, Navigation and Control Conference (CGNCC), Nanjing, 2016, pp. 1963-1968.

## Directory Structure
* **GPR_DOB** contains the code for gaussian process regression model with derivative observations. This is an exact replicate of the repository [gpr_dob](https://github.com/teancake/gpr_dob) with an additional 'sq_dist.c' and its compiled binary files to improve the execution speed of the original mfile 'sq_dist.m'
* **GPR_DOB_MAP** contains the main code for this repository, i.e. the MAP estimator using GPR model and derivative observations.

* Files with a name of **example_** are example files. 

## Examples on Aircraft Angle of Attack Estimation 
There are two examples in this repository, both run in MATLAB. The examples estimate the angle of attack of an aircraft using measurements of other longitudinal vairables, and constitutes the case study investigated in my CGNCC2016 paper. The first example uses offline data collected from the aircraft simulation model (which is a Simulink model), thus can be executed without Simulink. The other one uses Simulink and provices a more complete simulation environment.

### Example using offline data without Simulink 
This example includes a single m-file 'example_gpr_dob_map.m' and three data files: 'example_offline_data.mat' for the offline aircraft input output data,  	'example_GTM_IODATA.mat' and 'example_GTM_LINEAR_LON.mat' for the aircraft linearisation data used in MAP design. 

To run this example, type `example_gpr_dob_map.m` and press enter in MATLAB command window. Further information are also included in this m-file.

### Example using Simulink
This example is a bit complicated. The two data files 'example_GTM_IODATA.mat' and 'example_GTM_LINEAR_LON.mat' used in MAP design are also included in this example, which means that the two examples share the same MAP estimator design and parameters, but run on different data sources. The MAP estimator in this example is also packed in a Simulink block included in the file 'example_cgncc16_paper_gtm.slx'. This MAP block needs be used in combination with the GTM simulation model in order to obtain data from the aircraft. In detail, before running this example:
* Download the [GTM Simulation model] (https://github.com/nasa/GTM_DesignSim "GTM_DesignSim").
* Go to directory 'GTM_DesignSim/gtm_design', and create a new Simulink model with the name of 'gtm_design_map.slx'.
* Copy and paste the blocks in 'example_cgncc16_paper_gtm.slx' and 'GTM_DesignSim/gtm_design/gtm_design.slx' to the Simulink model 'gtm_design_map'. The figure below shows how the contents of the two simulink files should be combined.
* Copy and paste all files and folders in this repository into 'GTM_DesignSim/gtm_design/'.
* Run `example_cgncc16_paper.m` in MATLAB. 

Note: this example also uses an m-file named 'ekfd.m', which implements an extended Kalman filter. The only reason for this is to re-use existing code, users can implement their own Kalman filter without bothering with the EKF.

<div style="text-align:center"><img src="https://github.com/teancake/gpr_dob_map/blob/master/example_cgncc16_paper_simulink_blocks.png" alt="simulink"></div> 
