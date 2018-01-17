function [desired_x_dx_ddx_root, desired_R_w_dw_root, ... 
          desired_x_dx_ddx_l_sole, desired_x_dx_ddx_r_sole, desired_s_ds_dds, state] ...
      = stateMachine(w_H_root, s0, w_H_root_0, w_H_l_sole_0, w_H_r_sole_0, rootVelocity,...
                     feetActivation, root_displacement, ROBOT_DOF)
       
           
% Keep initial root position + a desired displacement
desired_x_root = w_H_root_0(1:3, 4) + root_displacement;
desired_x_dx_ddx_root = [desired_x_root, zeros(3,2)];

% Keep initial root orientation
desired_R_w_dw_root = [w_H_root_0(1:3,1:3), zeros(3,2)];

% Keep initial joint positions
desired_s_ds_dds = [s0, zeros(ROBOT_DOF,2)];

%Keep initial feet positions
desired_x_dx_ddx_l_sole = [w_H_l_sole_0 zeros(4,2)];
desired_x_dx_ddx_r_sole = [w_H_r_sole_0 zeros(4,2)];


%Define states
% both feet in contact and not walking
if feetActivation(1) && feetActivation(2)
    state = 1; %balancing two feet
    
    %Bring root over right foot
    desired_x_root = [w_H_r_sole_0(1:2,4);
                      w_H_root_0(3, 4)];
    desired_x_dx_ddx_root = [desired_x_root, zeros(3,2)];
    
    if norm(w_H_root(1:3, 4) - desired_x_root) < 0.01 %&& norm(rootVelocity) < 0.05
%         %this will not work except with sequential states. keep previous
%         %state in memory to go back straight here instead of checking error
%         %on CoM all the time
        %lift the left foot
        desired_x_l_sole = w_H_l_sole_0 + [zeros(4,3) [0;0;0.1;0]];
        desired_x_dx_ddx_l_sole = [desired_x_l_sole zeros(4,2)];
    end
    
% one foot in contact and not walking
elseif feetActivation(1) %feetActivation(2) < 0.9 %only left foot on ground
    state = 2; %balancing one foot
    %When the robot is stabilized, put the foot back down
    %stabilized means root acceleration/velocity is small
    if norm(rootVelocity) < 0.1
        state = 1;        
        % Bring root position back towards initial position
        desired_x_root = w_H_root_0(1:3, 4);
        desired_x_dx_ddx_root = [desired_x_root, zeros(3,2)];
    end
    
else %only right foot on ground
    state = 3;
    if rootVelocity < 0.1
        state = 1;
        % Bring root position back towards initial position
        desired_x_root = w_H_root_0(1:3, 4);
        desired_x_dx_ddx_root = [desired_x_root, zeros(3,2)];
    end
end

