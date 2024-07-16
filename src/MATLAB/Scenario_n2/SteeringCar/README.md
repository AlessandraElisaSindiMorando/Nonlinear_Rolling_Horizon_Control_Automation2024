# Steering Car Nonlinear MPC-MHE Simulation
The instructions are the [same](../../Scenario_n1/SteeringCar/README.md) of the first scenario, just the file to be run is **CasaDi_NMPC_MHE_UGV_Scenario2**.

All the content of the [SteeringCar](../../Scenario_n1/SteeringCar) folder should be copied and pasted in the current folder apart for the 
**NonLinear_MPC_with_Obstacles_CasaDi** MATLAB function. Indeed to deal with multiple obstacles the function has been rewritten, see 
**NonLinear_MPC_with_Obstacles_CasaDi_moreObstacles**.

The **results** folder contains the simualtion qualitative and quantitative results.