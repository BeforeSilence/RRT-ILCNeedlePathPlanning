# RRT-ILCNeedlePathPlanning
This repository includes reference code for RRT and ILC proragm used in needle-tissue interaction model based path planning

Reference Code for needle tissue interaction model based path planning
This document includes standalone RRT needle path planning programs along with corresponding RRT planning evaluation programs.
The main function for ILC path planning method based on the needle-tissue interaction model is also included; however, this program is not designed to run independently and is provided solely for reference.
All programs are developed with Matlab2019
For further information, please contact:

limurong@zhejianglab.com
shilun@zju.edu.cn

RRTRobustTest.m:
RRT Test Program. You can run this program directly to see the final path planning results of APF-Guided RRT with RD Selections.
There will 10 random scene be generated to test the RRT.

main_ILC.m:
This is the main Program for needle tissue interaction model based path planning with ILC correction.
This program cannot run without needle tissue interaction model. Thus, this program is only for Reference.

RRT_Tree.m:
The main program of our RRT algorithm. For readers interested in replicate the RRT path planning algorithm, you can refer to this part to find details.

EvaluateRRTPath.m:
Program for evaluating the RRT planned needle path.

randomInsertScene.m:
Program for generating random scene for RRT test.
