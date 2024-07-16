# Quadcopter Nonlinear MPC-MHE Simulation
The instructions are the [same](../../Scenario_n1/Quadcopter/README.md) of the first scenario, just the file to be run is **NMPC_UAV_MHE_CasaDi_Scenario2**.

All the content of the [SteeringCar](../../Scenario_n1/Quadcopter) folder should be copied and pasted in the current folder apart for the .mat files.

```shell
>> clear
>> load('SimulationParameters.mat')
>> load('XYThetaVPred_Scenario2.mat')
```

The **results** folder contains the simualtion qualitative and quantitative results.
