# MHE-NMPC UGV Results 
The instructions are the [same](../../Scenario_n1/SteeringCar/results/README.md) as for the first scenario.

Copy and paste all the files in the [results](../../Scenario_n1/SteeringCar/results) folder of the first test apart for the **ResultsScript** and the .mat file.

Run the **ResultsScript** script to plot all figures. The following text is displayed
```shell
Tracking Err.
RMSEx = 3.258398e-01 RMSEy = 1.936328e-01 RMSEtheta = 2.834659e-01 RMSEv = 9.554818e-02
Prediction Err.
RMSEx = 1.791083e-02 RMSEy = 1.624853e-02 RMSEtheta = 3.399088e-02 RMSEv = 1.526124e-02
Estimation Err.
RMSEx = 1.166335e-04 RMSEy = 6.355760e-05 RMSEtheta = 7.016448e-04 RMSEv = 5.133293e-04
>>
```

It's useful for the other simulation ot store the one-step predictions and the (x,y)-position of the UGV, 
see **XYThetaVPred_Scenario3** and **XSim_Scenario3**.
