# MHE-NMPC UGV Results 
The instructions are the [same](../../Scenario_n1/SteeringCar/results/README.md) as for the first scenario.

Copy and paste all the files in the [results](../../Scenario_n1/SteeringCar/results) folder of the first test apart for the **ResultsScript** and the .mat file.

Run the **ResultsScript** script to plot all figures. The following text is displayed
```shell
Tracking Err.
RMSEx = 2.171232e-01 RMSEy = 4.616422e-02 RMSEtheta = 2.031956e-01 RMSEv = 7.656478e-02
Prediction Err.
RMSEx = 1.498192e-02 RMSEy = 1.920266e-02 RMSEtheta = 3.463466e-02 RMSEv = 1.512093e-02
Estimation Err.
RMSEx = 2.907976e-05 RMSEy = 2.850032e-05 RMSEtheta = 6.935230e-04 RMSEv = 9.768503e-05
>>
```

It's useful for the other simulation ot store the one-step predictions and the (x,y)-position of the UGV, 
see **XYThetaVPred_Scenario4** and **XSim_Scenario4**.
