%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2016 CoDyCo
%  * @author: Daniele Pucci & Marie Charbonneau
%  * Permission is granted to copy, distribute, and/or modify this program
%  * under the terms of the GNU General Public License, version 2 or any
%  * later version published by the Free Software Foundation.
%  *
%  * A copy of the license can be found at
%  * http://www.robotcub.org/icub/license/gpl.txt
%  *
%  * This program is distributed in the hope that it will be useful, but
%  * WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  * Public License for more details
%  */
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; clc;
%% clearing functions with persistent variables
% % clear stateMachineWalking QpBalancingSOT
%% GENERAL SIMULATION INFO
% If you are simulating the robot with Gazebo, 
% remember that you have to launch Gazebo as follows:
% 
% gazebo -slibgazebo_yarp_clock.so
% 
% and set the environmental variable YARP_ROBOT_NAME = icubGazeboSim.
% To do this, you can uncomment the 

%setenv('YARP_ROBOT_NAME','iCubGenova02');
%setenv('YARP_ROBOT_NAME','iCubGenova04');
setenv('YARP_ROBOT_NAME','icubGazeboSim');

% Simulation time in seconds
CONFIG.SIMULATION_TIME     = inf;
% Simulation time step in seconds
CONFIG.Ts                  = 0.01; %  Controller period [s]

%% PRELIMINARY CONFIGURATIONS 
% SM.SM_TYPE: defines the kind of state machines that can be chosen. 
% 'WALKING': under development.
sm.SM_TYPE                 = 'WALKING';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONFIGURATIONS COMPLETED: loading gains and parameters for the specific robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DO NOT MODIFY THE FOLLOWING VARIABLES, THEY ARE AUTOMATICALLY 
%% CHANGED WHEN SIMULATING THE ROBOT ON GAZEBO, 
WBT_modelName            = 'matlabTorqueWalking';

FRAMES.BASE              = 'root_link';

% CONFIG.USE_IMU4EST_BASE: if set to false, the base frame is estimated by 
% assuming that either the left or the right foot stays stuck on the ground. 
% Which foot the controller uses depends on the contact forces acting on it. 
% If set to true, the base orientation is estimated by using the IMU, while
% the base position by assuming that the origin of either the right or the
% left foot does not move. 
CONFIG.USE_IMU4EST_BASE    = false;

% CONFIG.YAW_IMU_FILTER and CONFIG.PITCH_IMU_FILTER: when the flag
% CONFIG.USE_IMU4EST_BASE = true, then the orientation of the floating base is
% estimated as explained above. However, the foot is usually perpendicular
% to gravity when the robot stands on flat surfaces, and rotation about the
% gravity axis may be de to the IMU drift in estimating this angle. Hence,
% when either of the flags CONFIG.YAW_IMU_FILTER or CONFIG.PITCH_IMU_FILTER
% is set to true, then the yaw and/or pitch angles of the contact foot are
% ignored and kept equal to the initial values.
CONFIG.YAW_IMU_FILTER      = true;
CONFIG.PITCH_IMU_FILTER    = true;

% CONFIG.CORRECT_NECK_IMU: when set equal to true, the kinematics from the
% IMU and the contact foot is corrected by using the neck angles. If it set
% equal to false, recall that the neck is assumed to be in (0,0,0)
CONFIG.CORRECT_NECK_IMU    = true;

%CONFIG.USE_INVERSE_KINEMATICS: when set to true, use reference joint
%positions computed from integration-based inverse kinematics; when set to
%false, use desired joint positions from state machine.
CONFIG.USE_INVERSE_KINEMATICS = false;


%% 
addpath('./src/')
addpath('../utilityMatlabFunctions/')
run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/gains.m')); 

