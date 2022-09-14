# handover-simulation-in-qualnet

This project includes the C++ and MATLAB files for simulating the handover decision algorithms using the QualNet simulator (v. 7.1)
This simulation was configured to verify the gains of the ML based handover decision algorithm specified in the following research paper:

https://scholar.google.com/citations?view_op=view_citation&hl=en&user=il5kaFUAAAAJ&citation_for_view=il5kaFUAAAAJ:roLk4NBRz8UC

In the C++ file, layer3_lte.cpp, I implemented the standard handover decision algorithms 
and my proposed one which interacts with the MATLAB code in crossingtimeprediction.m to make predictions of the mobile device's mobility. 

The prediction is based on Bayesian regression, a probabilistic ML algorithm. At the beginning of the simulation,
the C++ commands the MATLAB to train the ML model using the code written in trainMybrm.m. 
cdft.m file describes the way of calculating cummulative probability distribution of Student's t-distribution.

Please note that the C++ file is part of the QualNet's LTE implementation that is commercial software and I modified the code. 
That is, I do not have authority to post the whole code of layer3_lte.cpp

So I just posted the functions that contains my modification

