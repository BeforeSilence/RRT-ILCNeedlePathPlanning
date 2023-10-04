%% DSL: This program is used for RRT Test
% The program will generate 10 random scene for RRT path planning
%%%%%%%%%%Variable Defination %%%%%%%%%%%%%%%
% corner: define a rectangular region as the feasible area for piercing the prosthetic device.
% num_spheres: define the number of target and obstacles.
% radius: define the random range of obstacles' radius.
% world: Store the region, target and obstacles information.
% R: the kinematic parameters of needle.
% initialPoint,spinning_z,L,arc_success: RRT Planned Control Parameters.
clear,clc;close all;
for i = 1:10
    %% Random Scene Gen
    corner = [0,0,-100;50,50,0];
    num_spheres = 6;
    radius = [3,10];
    world = randomInsertScene(corner, num_spheres, radius);
    %% RRT
    R = 100; % Needle Radius
    [initialPoint,spinning_z,L,arc_success,world] = RRT_Tree(world.obstacles(1,:),world.obstacles(2:end,:),R);
    % Evaluate RRT Planned Needle Path
    EvaluateRRTPath(world.obstacles(1,:),world.obstacles(2:end,:),initialPoint,spinning_z,L,arc_success,1);
end

