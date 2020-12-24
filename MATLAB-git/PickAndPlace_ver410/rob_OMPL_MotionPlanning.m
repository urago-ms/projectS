function [simTime] = rob_OMPL_MotionPlanning(clientID, sim, robotInitialState, rob_handle, fpos3_DummyPose, fpos4_DummyPose, fpos5_DummyPose, fpos6_DummyPose)

            % % %     Some parameters:
            % % % % %             approachVector = [0,0,1];      % often a linear approach is required. This should also be part of the calculations when selecting an appropriate state for a given pose
            approachVector = [0,0,0.1];      % often a linear approach is required. This should also be part of the calculations when selecting an appropriate state for a given pose
            approachVector_0 = [0,0,0];      % often a linear approach is required. This should also be part of the calculations when selecting an appropriate state for a given pose
            approachVector_up = [0,0,-0.1];      % often a linear approach is required. This should also be part of the calculations when selecting an appropriate state for a given pose
            
            maxConfigsForDesiredPose = 10; % we will try to find 10 different states corresponding to the goal pose and order them according to distance from initial state
            maxTrialsForConfigSearch = 300;    % a parameter needed for finding appropriate goal states
            searchCount = 1;       % how many times OMPL will run for a given task
            minConfigsForPathPlanningPath = 200;   % interpolation states for the OMPL path
            minConfigsForIkPath = 100; % interpolation states for the linear approach path
            collisionChecking = 1; % whether collision checking is on or off
  
            % % %          Do the path planning here (between a start state and a goal pose, including a linear approach phase):
            inInts = [rob_handle, collisionChecking, minConfigsForIkPath, minConfigsForPathPlanningPath, maxConfigsForDesiredPose, maxTrialsForConfigSearch, searchCount];
            inFloats = horzcat(robotInitialState, fpos5_DummyPose, approachVector, approachVector_up);
            %             inFloats = horzcat(robotInitialState, target3Pose, target1Pose);
            %                         inFloats = horzcat(robotInitialState, TargetDummyPose);



            pathTime_tic = tic;

            [res, retInts, path, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                'remoteApiCommandServer', ...
                sim.sim_scripttype_childscript, ...
                'findPath_goalIsPose', ...
                inInts,inFloats, ...
                '', ...
                [], ...
                sim.simx_opmode_oneshot_wait);
            
            if (res==0) && ~isempty(path)
                % % %          The path could be in 2 parts: a path planning path, and a linear approach path:
%                 part1StateCnt = retInts(1);
%                 part2StateCnt = retInts(2);
%                 path1 = path(1:part1StateCnt*6);
                
                
                % % %                Visualize the first path:
                [res, retInts, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                    'remoteApiCommandServer', ...
                    sim.sim_scripttype_childscript, ...
                    'visualizePath', ...
                    [rob_handle,255,0,255],path, ...
                    '', ...
                    [], ...
                    sim.simx_opmode_oneshot_wait);
                
                line1Handle = retInts(1);
                
                % % %          Make the robot follow the path:
                [res, retInts, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                    'remoteApiCommandServer', ...
                    sim.sim_scripttype_childscript, ...
                    'runThroughPath', ...
                    [rob_handle],path, ...
                    '', ...
                    [], ...
                    sim.simx_opmode_oneshot_wait);
                
                exeTime_pathT = toc(pathTime_tic)
                
                
                % % %              Wait until the end of the movement:
                runningPath = true;
                while runningPath
                    %                     res,retInts,retFloats,retStrings,retBuffer=sim.simxCallScriptFunction(clientID,'remoteApiCommandServer',sim.sim_scripttype_childscript,'isRunningThroughPath',[robotHandle],[],[],emptyBuff,sim.simx_opmode_oneshot_wait)
                    
                    [res, retInts, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                        'remoteApiCommandServer', ...
                        sim.sim_scripttype_childscript, ...
                        'isRunningThroughPath', ...
                        [rob_handle],[], ...
                        '', ...
                        [], ...
                        sim.simx_opmode_oneshot_wait);
                    
                    runningPath = retInts(1) == 1;
                end
                
%                 path2 = path(part1StateCnt*6:end);
                
                % % %             Suck objects
                [res, retPath, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                    'IRB4600_connection', ...
                    sim.sim_scripttype_childscript, ...
                    'suck_object', ...
                    [],[], ...
                    '', ...
                    [], ...
                    sim.simx_opmode_blocking);
                
                
                
                % % % % %                 Get the robot current state:
                %                         res,retInts,robotCurrentConfig,retStrings,retBuffer=sim.simxCallScriptFunction(clientID,'remoteApiCommandServer',sim.sim_scripttype_childscript,'getRobotState',[robotHandle],[],[],emptyBuff,sim.simx_opmode_oneshot_wait)
                
                [res, retInts, robotCurrentConfig, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                    'remoteApiCommandServer', ...
                    sim.sim_scripttype_childscript, ...
                    'getRobotState', ...
                    [rob_handle],[], ...
                    '', ...
                    [], ...
                    sim.simx_opmode_oneshot_wait);
                
                                         % % %          Do the path planning 2 here (between a start state and a goal pose, including a linear approach phase):
                approachVector2 = [0,0,-0.1];
                
                inInts_2 = [rob_handle, collisionChecking, minConfigsForIkPath, minConfigsForPathPlanningPath, maxConfigsForDesiredPose, maxTrialsForConfigSearch, searchCount];
                inFloats_2 = horzcat(robotCurrentConfig, fpos3_DummyPose, approachVector_0);
                % inFloats = horzcat(robotCurrentConfig, target3Pose, target1Pose);

                
                
                
                % % %         Get simtime (start time)
                [res_time, retInts_time, retFloats_time, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                    'ResizableFloor_5_25', ...
                    sim.sim_scripttype_childscript, ...
                    'get_simtime', ...
                    [],[], ...
                    '', ...
                    [], ...
                    sim.simx_opmode_blocking);
                
                [res, retInts, path, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                    'remoteApiCommandServer', ...
                    sim.sim_scripttype_childscript, ...
                    'findPath_goalIsPose', ...
                    inInts_2,inFloats_2, ...
                    '', ...
                    [], ...
                    sim.simx_opmode_oneshot_wait);
                
                if (res==0) && ~isempty(path)
                    
                    % % %                Visualize the first path:
                    [res, retInts, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                        'remoteApiCommandServer', ...
                        sim.sim_scripttype_childscript, ...
                        'visualizePath', ...
                        [rob_handle,255,0,255],path, ...
                        '', ...
                        [], ...
                        sim.simx_opmode_oneshot_wait);
                    
                    line2Handle = retInts(1);
                    
                    % % %          Make the robot follow the path:
                    [res, retInts, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                        'remoteApiCommandServer', ...
                        sim.sim_scripttype_childscript, ...
                        'runThroughPath', ...
                        [rob_handle],path, ...
                        '', ...
                        [], ...
                        sim.simx_opmode_oneshot_wait);
                    
                    
                    % % %              Wait until the end of the movement:
                    runningPath = true;
                    while runningPath
                        %                     res,retInts,retFloats,retStrings,retBuffer=sim.simxCallScriptFunction(clientID,'remoteApiCommandServer',sim.sim_scripttype_childscript,'isRunningThroughPath',[robotHandle],[],[],emptyBuff,sim.simx_opmode_oneshot_wait)
                        
                        [res, retInts, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                            'remoteApiCommandServer', ...
                            sim.sim_scripttype_childscript, ...
                            'isRunningThroughPath', ...
                            [rob_handle],[], ...
                            '', ...
                            [], ...
                            sim.simx_opmode_oneshot_wait);
                        
                        runningPath = retInts(1) == 1;
                    end
                    
                    % % %             Release objects
                    [res, retPath, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                        'IRB4600_connection', ...
                        sim.sim_scripttype_childscript, ...
                        'release_object', ...
                        [],[], ...
                        '', ...
                        [], ...
                        sim.simx_opmode_blocking);
                    
                    
                                    % % %         Get simtime (Finish time)
                [res_time, retInts_time, retFloats_time2, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                    'ResizableFloor_5_25', ...
                    sim.sim_scripttype_childscript, ...
                    'get_simtime', ...
                    [],[], ...
                    '', ...
                    [], ...
                    sim.simx_opmode_blocking);
                
                            simTime = retFloats_time2(1) - retFloats_time(1)

                    
                    
                    % % %             Remove path
                    [res, retPath, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                        'remoteApiCommandServer', ...
                        sim.sim_scripttype_childscript, ...
                        'removeLine', ...
                        [line1Handle],[], ...
                        '', ...
                        [], ...
                        sim.simx_opmode_blocking);
                    [res, retPath, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                        'remoteApiCommandServer', ...
                        sim.sim_scripttype_childscript, ...
                        'removeLine', ...
                        [line2Handle],[], ...
                        '', ...
                        [], ...
                        sim.simx_opmode_blocking);
                    
                end
                
                
                
                
                
                % % % % %                                 % % %      Visualize the second path (the linear approach):
                % % % % %                 % % %              Wait until the end of the movement:

           
                
                
            end
            

end