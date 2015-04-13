clear all;
clc;

robotName = 'icubGazeboSim';            
localName = 'matlabTorqueBalancing';

simulationTime    = inf;       % Simulation time in seconds

activeFeetConstraints    = [1 1];

DEMO_LEFT_AND_RIGHT      = 1;  % Either 0 or 1 
noOscillationTime        = 0; % If DEMO_LEFT_AND_RIGHT = 1, the variable noOscillationTime is the time, in seconds, 
                               % that the robot waits before starting the left-and-righ

DEMO_MOVING_LEG_AND_ARMS = 0; 

 

% Controller period
Ts                = 0.01; % [s]

% Load gains and parameters for the specific robot
run(strcat('robots/',getenv('YARP_ROBOT_NAME'),'/gains.m')); 

[ConstraintsMatrix,bVectorConstraints]= constraints(forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,footSize,fZmin);
