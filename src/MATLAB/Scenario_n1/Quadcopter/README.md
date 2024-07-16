# Quadcopter Nonlinear MPC-MHE Simulation
To run the simulation, open and run the  **NMPC_MHE_UAV_CasaDi_Scenario1** Simulink file.
Before running, run the following command to load all parameters defined in the Simulink model referring to the drone, 
the reference,
the Optitrack mocap, 
the Nonlinear MPC MATLAB object, 
and the one-step prediction of the ground robot obtained from the previous simulation

$$[x(k+1|k) \  y(k+1|k) \  \theta(k+1|k) \ v(k+1|k)]^T \quad k = 0,1,\cdots$$

For more details see files **UAV_parameters**, **Optitrack_parameters**,
**UAV_nmpcobj**.

```shell
>> clear
>> load('SimulationParameters.mat')
>> load('XYThetaVPred_wObstacles.mat')
```

The parameters have been chosen according to the data sheet of the [Optitrack](https://optitrack.com/applications/virtual-production/) mocap system and the[Parrot BEBOP 2.0](https://www.parrot.com/assets/s3fs-public/2021-09/bebop-2_user-guide_uk.pdf) drone.

The Nonlinear Model Predictive Controller (NMPC) MATLAB object definition uses the MATLAB functions **UAVStateFnc.m**, **UAVStateJacobianFcn**, **UAVOutputFcn**, and **UAVOutputJacobianFcn**.

Moreover, as Moving Horizon Estimator (MHE) requires the online solution of an optimization problem, the IPOPT [CasADi](https://web.casadi.org/) solver is used. Hence the right distribution should be downloaded from the CasaDi webpage [here](https://web.casadi.org/get/), then unzipped and stored in a folder whose path should be specified in the MATLAB function **MHE_CasaDi**. 
```
function [...] = ...
    ...
% Import CasADi files. 
% ! Path **TO BE CHANGED** according to the folder structures and the
% distribution (see https://web.casadi.org/get/)
addpath('../../casadi-3.6.5-linux64-matlab2018b/');
import casadi.*
    ...
end
```
The **results** folder contains the simualtion qualitative and quantitative results.
