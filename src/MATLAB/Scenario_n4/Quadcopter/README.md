# Quadcopter Nonlinear MPC-MHE Simulation
The instructions are the [same](../../Scenario_n1/Quadcopter/README.md) of the first scenario.
For the simulation, the [Simulink model](../../Scenario_n3/Quadcopter/NMPC_UAV_MHE_CasaDi_Scenario3.slx) of the third case study can be used by loading the local **SimulationParameters**.

All the content of the [SteeringCar](../../Scenario_n1/Quadcopter) folder should be copied and pasted in the current folder apart for the .mat files.

```shell
>> clear
>> load('SimulationParameters.mat')
>> load('XYThetaVPred_Scenario4.mat')
```

The **results** folder contains the simualtion qualitative and quantitative results.
