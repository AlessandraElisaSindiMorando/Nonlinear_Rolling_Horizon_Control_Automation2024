# MHE-NMPC UGV Results 
The instructions are the [same](../../Scenario_n1/SteeringCar/results/README.md) as for the first scenario.

Copy and paste all the files in the [results](../../Scenario_n1/SteeringCar/results) folder of the first test apart for the **ResultsScript** and the .mat file.

Run the **ResultsScript** script to plot all figures. The following text is displayed
```shell
Tracking Err.
RMSEx = 2.318173e-01 RMSEy = 7.435455e-02 RMSEtheta = 2.167486e-01 RMSEv = 8.210322e-02
Prediction Err.
RMSEx = 1.657553e-02 RMSEy = 1.607084e-02 RMSEtheta = 3.208522e-02 RMSEv = 1.500973e-02
Estimation Err.
RMSEx = 5.963849e-05 RMSEy = 3.628341e-05 RMSEtheta = 6.547107e-04 RMSEv = 2.302710e-04
>>
```

It's useful for the other simulation ot store the one-step predictions and the (x,y)-position of the UGV, 
see **XYThetaVPred_Scenario2** and **XSim_Scenario2**.
