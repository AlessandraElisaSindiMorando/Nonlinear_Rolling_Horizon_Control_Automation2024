# Steering Car Nonlinear MPC-MHE Simulation
To run the simulation, open and run the  **CasaDi_NMPC_MHE_UGV_circle2m** Simulink file.
Before running, run the following command to load all parameters defined in the Simulink model.
```shell
>>clear
>>load('SimulationParameters.mat')
```
The .mat file constains both the parameters of the ground robot (see file **UGV_parameters**) and the Optitrack system (see file **Optitrack_parameters**) 
The parameters have been chosen according to the data sheet of the [Optitrack](https://optitrack.com/applications/virtual-production/) mocap system and the [JetRacer](https://developer.nvidia.com/embedded/community/jetson-projects/jetracer) racecar.

Moreover, as Nonlinear Model Predictive Controller (MPC) and Moving Horizon Estimator (MHE) requires the online solution of an optimization problem, the IPOPT [CasADi](https://web.casadi.org/) solver is used. Hence the right distribution should be downloaded from the CasaDi webpage [here](https://web.casadi.org/get/), then unzipped and stored in a folder whose path should be specified in both MATLAB functions **NonLinear_MPC_with_Obstacles_CasaDi** and **MHE_RK_CasaDi**. 
```
function [...] = ...
    ...
% Import CasADi files. 
% ! Path **TO BE CHANGED** according to the folder structures and the
% distribution (see https://web.casadi.org/get/)
addpath('../../casadi-3.6.5-linux64-matlab2018b/')
import casadi.*
    ...
end
```
The **results** folder contains the simualtion qualitative and quantitative results.
