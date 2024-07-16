# MHE-NMPC UGV Results 
Once it has been run the Simulink model, in the out variable are stored all data from the simulation.
The out variable has been saved in the **out_Scenario1** file.

Run the **ResultsScript** script to plot all figures. The following text is displayed

```shell
Tracking Err.
RMSEx = 5.294520e-02 RMSEy = 1.597007e-01 RMSEtheta = 1.525843e-01 RMSEv = 5.769119e-02
Prediction Err.
RMSEx = 1.550969e-02 RMSEy = 1.570374e-02 RMSEtheta = 2.387440e-02 RMSEv = 1.059806e-02
Estimation Err.
RMSEx = 3.243160e-05 RMSEy = 4.463210e-05 RMSEtheta = 6.488544e-04 RMSEv = 1.727660e-04
>>
```

It's useful for the other simulation ot store the one-step predictions and the (x,y)-position of the UGV, 
see **XYThetaVPred_Scenario1** and **XSim_Scenario1**.
