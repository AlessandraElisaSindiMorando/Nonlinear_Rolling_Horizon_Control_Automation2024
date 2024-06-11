# MHE-NMPC UGV Results 
Once it has been run the Simulink model, in the out variable are stored all data from the simulation.
The out variable has been saved in the **out** file.

Run the **ResultsScript** script to plot all figures. The .fig file have been obtained by running the indicated file.
<p align="center">
<img src="Figure5.eps">
</p>
The following text is also displayed
```shell
Tracking Err.
RMSEx = 4.126892e-02 RMSEy = 1.327781e-01 RMSEtheta = 1.302032e-01 RMSEv = 5.438888e-02
Prediction Err.
RMSEx = 1.597134e-02 RMSEy = 1.563471e-02 RMSEtheta = 2.171084e-02 RMSEv = 1.100832e-02
Estimation Err.
RMSEx = 3.077111e-05 RMSEy = 4.789285e-05 RMSEtheta = 6.477122e-04 RMSEv = 1.817628e-04
>>
```
It's useful for the other simulation ot store the one-step predictions of the (x,y)-position of the UGV, see **XYThetaVPred_wObstacles**
