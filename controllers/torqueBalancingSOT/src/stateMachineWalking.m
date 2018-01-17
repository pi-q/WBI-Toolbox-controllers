function [w_H_b, CoMDes, qDes, feetActivation, impedances, dampings, PDgainsPos,PDgainsRot, currentState, jointsSmoothingTime,...
          desRootRotPosVelAcc, desLFootRotPosVelAcc, desLFootOrigin, desRFootRotPosVelAcc, desRFootOrigin] ...
       = stateMachineWalking(qj, qj0, CoM_0, ...
                             l_sole_CoM, r_sole_CoM, l_sole_H_b, r_sole_H_b, rotFrame_H_b0, ...
                             wrench_leftFoot, wrench_rightFoot, sm, gain, CONFIG)
    persistent state;
    persistent stateTime;
    persistent w_H_fixedLink;
    persistent w_H_r_sole0;
    persistent l_sole_H_b0;
    
    if isempty(state) || isempty(stateTime) || isempty(w_H_fixedLink) || isempty(w_H_r_sole0) || isempty(l_sole_H_b0) 
        state            = sm.stateAt0;
        stateTime        = 0;
        w_H_fixedLink    = eye(4);
        w_H_r_sole0      = w_H_fixedLink * l_sole_H_b / r_sole_H_b; %l_sole_H_b / r_sole_H_b;
        l_sole_H_b0      = l_sole_H_b;
    end
    
    CoMDes               = CoM_0; %CoM_0 represents initial value of w_H_CoM
    feetActivation       = [1; 1];
    qDes                 = qj0;
    w_H_b                = eye(4);

    l_sole_H_rotFrame0   = l_sole_H_b0 / rotFrame_H_b0;
    desRootRotPosVelAcc  = [l_sole_H_rotFrame0(1:3,1:3),zeros(3,2)];
    desLFootRotPosVelAcc = [eye(3),zeros(3,2)];
    desLFootOrigin       = zeros(3,1);
    desRFootRotPosVelAcc = [eye(3),zeros(3,2)];
    desRFootOrigin       = w_H_r_sole0(1:3,4);
    
        %Feet are considered in contact with the ground by default.
    %Contact switching happens at foot liftoff and foot touchdown. No contact is considered: 
    %once the vertical wrench acting on this foot is below a threshold (foot liftoff)
    %and as long as the vertical wrench acting on this foot is  below a threshold (foot touchdown)
    feetActivation       = [1; 1];
    if (state <  4) && wrench_rightFoot(3) < sm.wrench.thresholdContactOff || ...
       (state >= 4) && wrench_rightFoot(3) < sm.wrench.thresholdContactOn
        feetActivation(2) = 0;
    elseif (state <  9) && wrench_leftFoot(3) < sm.wrench.thresholdContactOff || ...
           (state >= 9) && wrench_leftFoot(3) < sm.wrench.thresholdContactOn
        feetActivation(1) = 0;
    end
    
    %% STATES
    % 1 - Two feet balancing
    % 2 - Transition to left foot
    % 3 - Left foot balancing
    % 4 - Preparing for switching
    % 5 - Looking for contact
    % 6 - Transition to initial position
    % 7 - Transition to right foot
    % 8 - Right foot balancing
    % 9 - Preparing for switching
    %10 - Looking for contact
    %11 - Transition to initial position
 
    %% 1 - TWO FEET BALANCING
    if state == 1 
        w_H_b             = w_H_fixedLink * l_sole_H_b;

        fixed_link_CoMDes = w_H_fixedLink \ [CoMDes;1];
        CoMError          = fixed_link_CoMDes(1:3) - l_sole_CoM(1:3);
        
        stateTime = stateTime + CONFIG.Ts;
        
        if norm(CoMError) < sm.com.threshold && stateTime > (sm.tBalancing) %after tBalancing time start next state
           state = 2;
           if sm.demoOnlyRightFoot
                state     = 6; 
                stateTime = 0;
           end
           
        end
    end
    
    %% 2 - TRANSITION TO THE LEFT FOOT
    if state == 2 
        w_H_b             = w_H_fixedLink * l_sole_H_b;      
        
        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes            = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';   
        qDes              = sm.joints.states(state,:)';

        fixed_link_CoMDes = w_H_fixedLink\[CoMDes;1];
        CoMError          = fixed_link_CoMDes(1:3) - l_sole_CoM(1:3);
       
        if norm(CoMError) < sm.com.threshold && ~feetActivation(2) %wrench_rightFoot(3) < sm.wrench.thresholdContactOff
           state          = 3; 
        end
    end
    
    %% 3 - LEFT FOOT BALANCING 
    if state == 3 
        w_H_b          = w_H_fixedLink * l_sole_H_b;
%         feetActivation    = [1; 0]; %right foot is no longer a constraint        

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes         = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        qDes           = sm.joints.states(state,:)';
        desRFootOrigin = w_H_fixedLink(1:3,4) + sm.origin.rightFoot(state,:)';
        
        %if not using feet position as a constraint, but only relying on postural
        %task, then the error on leg posture shall be used
        if CONFIG.QP.USE_STRICT_TASK_PRIORITIES_NO_FOOT_ACCELERATION
            qTildeRLeg  = qj(end-5:end) - qDes(end-5:end);  
            qTildeLLeg  = qj(end-11:end-6) - qDes(end-11:end-6);        
            if norm(qTildeRLeg)*180/pi < sm.joints.thresholdNotInContact && norm(qTildeLLeg)*180/pi < sm.joints.thresholdInContact
                state   = 4;
            end
        else %otherwise, use the error on foot position
            RFootOrigin  = w_H_b / r_sole_H_b;
            r_foot_error = RFootOrigin(1:3, 4) - desRFootOrigin;
            if norm(r_foot_error) < sm.foot.threshold
                state    = 4;
            end
        end      
    end    
    
    %% 4 - PREPARING FOR SWITCHING
    if state == 4 
        w_H_b          = w_H_fixedLink * l_sole_H_b;
%         feetActivation    = [1; 0]; %right foot is no longer a constraint 
            
        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes         = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        qDes           = sm.joints.states(state,:)';  
%         desRFootOrigin = w_H_fixedLink(1:3,4) + sm.origin.rightFoot(state,:)';
        desRFootOrigin_fromState = w_H_fixedLink(1:3,4) + sm.origin.rightFoot(state,:)';
        desRFootOrigin = [desRFootOrigin(1); desRFootOrigin(2); desRFootOrigin_fromState(3)];
        
        %if not using feet position as a constraint, but only relying on postural
        %task, then the error on leg posture shall be used
        if CONFIG.QP.USE_STRICT_TASK_PRIORITIES_NO_FOOT_ACCELERATION
            qTildeRLeg  = qj(end-5:end) - qDes(end-5:end);  
            qTildeLLeg  = qj(end-11:end-6) - qDes(end-11:end-6);        
            if norm(qTildeRLeg)*180/pi < sm.joints.thresholdNotInContact && norm(qTildeLLeg)*180/pi < sm.joints.thresholdInContact
                state   = 5;
            end
        else %otherwise, use the error on foot position
            RFootOrigin  = w_H_b / r_sole_H_b;
            r_foot_error = RFootOrigin(1:3, 4) - desRFootOrigin;
        
            if norm(r_foot_error) < sm.foot.threshold
                state    = 5;
            end
        end      
    end
    
    %% 5 - LOOKING FOR A CONTACT
    if state == 5
        w_H_b          =  w_H_fixedLink * l_sole_H_b;
%         feetActivation    = [1; 0]; %right foot is no longer a constraint
     
        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes         = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        qDes           = sm.joints.states(state,:)';
%         desRFootOrigin = desRFootOrigin; % w_H_fixedLink(1:3,4) + sm.origin.rightFoot(state,:)';
        desRFootOrigin_fromState = w_H_fixedLink(1:3,4) + sm.origin.rightFoot(state,:)';
        desRFootOrigin = [desRFootOrigin(1); desRFootOrigin(2); desRFootOrigin_fromState(3)];

        if feetActivation(2) %wrench_rightFoot(3) > sm.wrench.thresholdContactOn
            state      = 6;
        end
    end
    
    %% TRANSITION TO INITIAL POSITION
    if state == 6 
        w_H_b             =  w_H_fixedLink * l_sole_H_b;
        
        fixed_link_CoMDes = w_H_fixedLink\[CoMDes;1];
        CoMError          = fixed_link_CoMDes(1:3) - l_sole_CoM(1:3);
        
        if norm(CoMError) < sm.com.threshold
            state         = 7;
            w_H_fixedLink = w_H_fixedLink*l_sole_H_b/r_sole_H_b;
        end
    end
    
    %% TRANSITION TO THE RIGHT FOOT
    if state == 7 
        w_H_b             = w_H_fixedLink * r_sole_H_b;
        
        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes            = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        qDes              = sm.joints.states(state,:)';
         
        fixed_link_CoMDes = w_H_fixedLink\[CoMDes;1];       
        CoMError          = fixed_link_CoMDes(1:3) - r_sole_CoM(1:3);

        if norm(CoMError) < sm.com.threshold && ~feetActivation(1) %wrench_leftFoot(3) < sm.wrench.thresholdContactOff
           state          = 8; 
        end

    end   
    
    %% 8 - RIGHT FOOT BALANCING 
    if state == 8 
        w_H_b          =  w_H_fixedLink * r_sole_H_b;
%         feetActivation = [0; 1]; %left foot is no longer a constraint  

        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes         = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        qDes           = sm.joints.states(state,:)';
        desLFootOrigin = w_H_fixedLink(1:3,4) + sm.origin.leftFoot(state,:)';
        
        %if not using feet position as a constraint, but only relying on postural
        %task, then the error on leg posture shall be used
        if CONFIG.QP.USE_STRICT_TASK_PRIORITIES_NO_FOOT_ACCELERATION
            qTildeRLeg  = qj(end-5:end) - qDes(end-5:end);        
            qTildeLLeg  = qj(end-11:end-6) - qDes(end-11:end-6); 
            if norm(qTildeLLeg)*180/pi < sm.joints.thresholdNotInContact && norm(qTildeRLeg)*180/pi < sm.joints.thresholdInContact
                state   = 9;
            end
        else %otherwise, use the error on foot position
            l_footOrigin   = w_H_b / l_sole_H_b;
            l_foot_error   = l_footOrigin(1:3, 4) - desLFootOrigin;
            if norm(l_foot_error) < sm.foot.threshold
                state    = 9;
            end
        end
    end
        
    %% 9 - PREPARING FOR SWITCHING
    if state == 9 
        w_H_b       = w_H_fixedLink * r_sole_H_b;
%         feetActivation = [0; 1]; %left foot is no longer a constraint        
    
        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes      = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        qDes        =  sm.joints.states(state,:)';  
%         desLFootOrigin = w_H_fixedLink(1:3,4) + sm.origin.leftFoot(state,:)';
        desLFootOrigin_fromState = w_H_fixedLink(1:3,4) + sm.origin.leftFoot(state,:)';
        desLFootOrigin = [desLFootOrigin(1); desLFootOrigin(2); desLFootOrigin_fromState(3)];

        %if not using feet position as a constraint, but only relying on postural
        %task, then the error on leg posture shall be used
        if CONFIG.QP.USE_STRICT_TASK_PRIORITIES_NO_FOOT_ACCELERATION
            qTildeRLeg  = qj(end-5:end) - qDes(end-5:end);        
            qTildeLLeg  = qj(end-11:end-6) - qDes(end-11:end-6); 
            if norm(qTildeLLeg)*180/pi < sm.joints.thresholdNotInContact && norm(qTildeRLeg)*180/pi < sm.joints.thresholdInContact
                state   = 10;
            end
        else %otherwise, use the error on foot position
            l_footOrigin   = w_H_b / l_sole_H_b;
            l_foot_error   = l_footOrigin(1:3, 4) - desLFootOrigin;
            
            if norm(l_foot_error) < sm.foot.threshold
                state    = 10;
            end
        end
    end
    
    %% 10 - LOOKING FOR A CONTACT
    if state == 10
        w_H_b          =  w_H_fixedLink * r_sole_H_b;
%         feetActivation    = [0; 1]; %left foot is no longer a constraint
        
        % Set the center of mass projection onto the x-y plane to be
        % coincident to the origin of the left foot (l_sole) plus a
        % configurable delta
        CoMDes         = [w_H_fixedLink(1:2,4);CoM_0(3)] + sm.com.states(state,:)';         
        qDes           = sm.joints.states(state,:)';
%         desLFootOrigin = desLFootOrigin; % w_H_fixedLink(1:3,4) + sm.origin.leftFoot(state,:)';
        desLFootOrigin_fromState = w_H_fixedLink(1:3,4) + sm.origin.leftFoot(state,:)';
        desLFootOrigin = [desLFootOrigin(1); desLFootOrigin(2); desLFootOrigin_fromState(3)];

        if feetActivation(1) %wrench_leftFoot(3) > sm.wrench.thresholdContactOn 
            state      = 11;
        end
    end
    
    %% TRANSITION TO INITIAL POSITION
    if state == 11 
        w_H_b                  = w_H_fixedLink * r_sole_H_b;
        
        fixed_link_CoMDes      = w_H_fixedLink\[CoMDes;1];
        CoMError               = fixed_link_CoMDes(1:3) - r_sole_CoM(1:3);
        
        if norm(CoMError) < sm.com.threshold
            if sm.demoInLoop
              state            = 2;
              w_H_fixedLink    = w_H_fixedLink*r_sole_H_b/l_sole_H_b;
              if sm.demoOnlyRightFoot
                 state         = 7;           
                 w_H_fixedLink = w_H_fixedLink*l_sole_H_b/r_sole_H_b;
              end
            end
        end
    end    
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    currentState        = state;
    
    impedances          = gain.joints.impedances(state,:);
    dampings            = gain.joints.dampings(state,:);
    
    PDgainsPos          = [gain.CoM.posPD(state,1:3)'  , gain.CoM.posPD(state,4:6)';
                           gain.lFoot.posPD(state,1:3)', gain.lFoot.posPD(state,4:6)';
                           gain.rFoot.posPD(state,1:3)', gain.rFoot.posPD(state,4:6)'];
    
%     PDgainsRot          = [gain.root.rotPD(state,:);
%                            gain.lFoot.rotPD(state,:);
%                            gain.rFoot.rotPD(state,:)]; 
                       
   PDgainsRot           = [gain.root.rotPD( state,1)*ones(3,1), gain.root.rotPD( state,2)*ones(3,1);
                           gain.lFoot.rotPD(state,1)*ones(3,1), gain.lFoot.rotPD(state,2)*ones(3,1);
                           gain.rFoot.rotPD(state,1)*ones(3,1), gain.rFoot.rotPD(state,2)*ones(3,1)];
                     
    
    jointsSmoothingTime = sm.jointsSmoothingTimes(state);