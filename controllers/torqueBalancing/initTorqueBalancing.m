clear all;
clc;

robotName = 'icub';            
localName = 'matlabTorqueBalancing';

simulationTime    = inf;    % Simulation time in seconds

USE_QP_SOLVER    = 0;

LEFT_RIGHT_FOOT_IN_CONTACT  = [1 0];

DEMO_MOVEMENTS      = true;% Either true or false 

SMOOTH_DES_COM      = 0;    % If equal to one, the desired streamed values 
                            % of the center of mass are smoothed internally 
SMOOTH_DES_Q        = 1;    % If equal to one, the desired streamed values 
                            % of the postural tasks are smoothed internally 
SMOOTH_CONSTRAINTS  = 1;    % If equal to one, the desired streamed values 
                            % of the logic vector identifying the active 
                            % constraints are smoothed internally 

% PLEASE, use logical values (true or false) for the following variable

USE_SM              = false; % If equal to true, the (internal) state machine 
                            % will be used. The robot will switch from 2 feet 
                            % to 1 (left) foot

                           
Ts                  = 0.01; %  Controller period [s]


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONFIGURATIONS COMPLETED: loading gains and parameters for the specific robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

run(strcat('robots/',getenv('YARP_ROBOT_NAME'),'/gains.m')); 
addpath('extra/')
[ConstraintsMatrix,bVectorConstraints]= constraints(forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,gain.footSize,fZmin);


robotSpecificReferences = fullfile('robots',getenv('YARP_ROBOT_NAME'),'initRefGen.m');
run(robotSpecificReferences);
robotSpecificFSM = fullfile('robots',getenv('YARP_ROBOT_NAME'),'initStateMachine.m');
run(robotSpecificFSM);


% If you want to sync Gazebo and simulink, 
% remember that you have to launch gazebo as follow:
% 
% gazebo -slibgazebo_yarp_clock.so
%
% Also, open the subsystem "Synchronizer" in the simulonk model 
% "balancingOptTau.mdl" and comment the block "Real Time Syncronizer" and
% uncomment the block "ySynchronizer".