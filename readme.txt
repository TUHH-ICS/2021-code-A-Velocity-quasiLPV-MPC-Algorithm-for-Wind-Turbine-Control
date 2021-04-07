This folder contains the files to reproduce the results and figures presented in

A. Dittmer, B. Sharan and H. Werner, "A Velocity quasiLPV-MPC Algorithm for 
Wind Turbine Control", 2021 European Control Conference (ECC) 

The figures can be reproduced using the file 'runGeneratePicsPaper.m'. 
The data for Figure 1 is generated using 'compareLinearModels.m' in folder 
'LinMdl'. Figure 2 is plotted with 'runCompareModels.m' and for Figures 3 
and 4 using 'runCompareCtrl.m'. Both files are in the folder 'NonLinMdl'.

In order to run the code, matfiles contained in folder dataIn are used. 
They were generated with
- FASTTool (https://github.com/TUDelft-DataDrivenControl/FASTTool)

The code has been tested using:
- Matlab R2019b 
It requires MATLAB, Simulink, and Control System Toolbox.

The Simulink project file 'Ecc21.prj' is provided as well as the necessary 
'resources' folder.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
FITNESS FOR A PARTICULAR PURPOSE.
