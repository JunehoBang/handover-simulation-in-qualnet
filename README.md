# handover-simulation-in-qualnet

This project includes the C++ and MATLAB files for simulating the handover decision algorithms using the QualNet simulator (v. 7.1)

In the C++ file, layer3_lte.cpp, I implemented the standard handover decision algorithms 
and my proposed one which interacts with the MATLAB code in crossingtimeprediction.m to make predictions of the mobile device's mobility. 

The prediction is based on Bayesian regression, a probabilistic ML algorithm. At the beginning of the simulation,
the C++ commands the MATLAB to train the ML model using the code written in trainMybrm.m. 
cdft.m file describes the way of calculating cummulative probability distribution of Student's t-distribution.
