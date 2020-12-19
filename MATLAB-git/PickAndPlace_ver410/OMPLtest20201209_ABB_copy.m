% This example illustrates how to execute complex commands from
% a remote API client. You can also use a similar construct for
% commands that are not directly supported by the remote API.
%
% Load the demo scene 'remoteApiCommandServerExample.ttt' in CoppeliaSim, then
% start the simulation and run this program.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!



function spawn_and_grasp()
% Timer start
tStart = tic;


disp('Program started');
% sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
sim = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1',19998,true,true,5000,5);

% % % Can start and stop simulations from matlab.
% % % Continuous api connection using port 19998.

if (clientID>-1)
    disp('Connected to remote API server');
    % % %     Simulation Start
    [res_startSim] = sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)
    
    %     exeTime_array_2 = zeros(1000000,2);
    exeTime_array = zeros(1,2);
    simTime_array = zeros(1,2);
    TimeToRestart_array = zeros(1,2);
    
    % % %     An array to store the processing time per session (Matlab Time)
    time = datestr(datetime,'yyyymmdd_HHMMSS');
    exeTime_file_name_csv = strcat(time, 'exeTime_array.csv');
    
    exeTime_array_cap = ["Repetition rate", "exeTime[s]"];
    writematrix(exeTime_array_cap, exeTime_file_name_csv);
    
    % % %     An array to store the processing time per session(Coppeliasim, Objective function)
    %     time = datestr(datetime,'yyyymmdd_HHMMSS');
    simTime_file_name_csv = strcat(time, 'simTime_array.csv');
    
    simTime_array_cap = ["Repetition rate", "Coppelia_simTime[s]"];
    writematrix(simTime_array_cap, simTime_file_name_csv);
    
    % % %     An array to store Time to Restart
    %     time = datestr(datetime,'yyyymmdd_HHMMSS');
    TimeToRestart_file_name_csv = strcat(time, 'TimeToRestart.csv');
    
    TimeToRestart_array_cap = ["count_overall", "TimeToRestart[s]"];
    writematrix(TimeToRestart_array_cap, TimeToRestart_file_name_csv);
    
    
    % % %     The number of iterations is "rep_overall x rep"
    rep_overall = 1;
    % % %     "rep times" units
    rep = 1000;
    
    % % %     Repetition overall
    for count_overall = 1:rep_overall
        
        % % %             Genarate a conveyor
        [res_con_genetate, con_handle] = sim.simxLoadModel(clientID,'customizable_conveyor_belt_06x2x08_fix_sensorPos.ttm', 0, sim.simx_opmode_blocking);
        [res_con_setpos] = sim.simxSetObjectPosition(clientID, con_handle, -1, [0 1 0.75], sim.simx_opmode_oneshot);
        [res_con_setorien] = sim.simxSetObjectOrientation(clientID, con_handle, -1, [0 0 -pi/2], sim.simx_opmode_oneshot);
        
        
        
        
        
        
        % % %     Repetition in "rep times" units
        % % %         Repeated creation and deletion of the device.
        for count = 1:rep
            
            
            % % %         Enable logging
            diary command_window.txt;
            disp("///////////////////////////////////////////////");
            
            exeTime_tic = tic;
            
            
            % % % % %         Display the number of repetitions in CoppeliaSim
            if (count_overall == 1)
                rep_rate = count;
            else
                rep_rate = (count_overall - 1)*rep + count;
            end
            
            [res_print_repetition_rate, retPath, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                'ResizableFloor_5_25', ...
                sim.sim_scripttype_childscript, ...
                'print_repetition_rate', ...
                [rep_rate], ...
                [], ...
                '', ...
                [], ...
                sim.simx_opmode_blocking);
            
            disp(rep_rate);
            
            % % %                         Determine layout
            [m_pos_random, p_pos_random, c_pos_random] = random_research();
            rob_pos_2 = [m_pos_random(1), m_pos_random(2), 0.1];
            tab_pos_2 = [p_pos_random(1), p_pos_random(2), 0.75];
            
            
            
            % % %             Generate facilities using MATLAB functions % % % % %
            % % %                         Genarate a table
            [res_tab_genetate, tab_handle] = sim.simxLoadModel(clientID,'customizable_table_with_create_cube_func_05_05_08.ttm', 0, sim.simx_opmode_blocking);
            [res_tab_setpos] = sim.simxSetObjectPosition(clientID, tab_handle, -1, tab_pos_2, sim.simx_opmode_oneshot);
            
            
            
            
            %             [res_get_tab_handle, tab_handle] = sim.simxGetObjectHandle(clientID,'customizableTable', sim.simx_opmode_blocking);
            %             [res_tab_getpos, tab_pos_2] = sim.simxGetObjectPosition(clientID, tab_handle, -1, sim.simx_opmode_blocking)
            
            
            %
            %             % % % Create Collection environment
            %             [res_CreateCollec retInts_CreateCollec retFloats_CreateCollec retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
            %                 'ResizableFloor_5_25', ...
            %                 sim.sim_scripttype_childscript, ...
            %                 'createCollection', ...
            %                 [tab_handle],[], ...
            %                 '', ...
            %                 [], ...
            %                 sim.simx_opmode_blocking);
            % %             pause(10);
            
            
            
            
            %             [res_get_rob_handle, rob_handle] = sim.simxGetObjectHandle(clientID,'Cuboid', sim.simx_opmode_blocking);
            
            
            % % %                         Genarate a robot
            [res_rob_genetate, rob_handle] = sim.simxLoadModel(clientID,'KUKA_IRB4600.ttm', 0, sim.simx_opmode_blocking);
            [res_rob_setpos] = sim.simxSetObjectPosition(clientID, rob_handle, -1, rob_pos_2, sim.simx_opmode_oneshot);
            %         disp(rob_handle);
            
            %             [res_tip_dummy_handle, rob_handle] = sim.simxGetObjectHandle(clientID,'Cuboid', sim.simx_opmode_blocking);
            
            
            
            
            
            if rep_rate == 1
                % % % Create Collection
                [res_CreateCollec retInts_CreateCollec retFloats_CreateCollec retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
                    'ResizableFloor_5_25', ...
                    sim.sim_scripttype_childscript, ...
                    'create_Collection', ...
                    [rob_handle, tab_handle],[], ...
                    '', ...
                    [], ...
                    sim.simx_opmode_blocking);
                %             pause(10);
            end
            
            
            
            
            
            
            
            
            % % %             Setting of target dummy (If generate only target dummy)
            % % % % %             target_dummy_pos = [0.25, 0, 0.4];
            % % % % %             target_dummy_orien = [0, 0, -pi/2];
            
            target_dummy_pos = [1, 0, 0.4];
            target_dummy_orien = [0, pi, 0];
            
            
            target_dummy_PosOrien = [target_dummy_pos, target_dummy_orien];
            
            % % % % %             [res_target_gen, TargetDummyHandle] = sim.simxCreateDummy(clientID, 0.03, [], sim.simx_opmode_blocking);
            % % % % %             [res_target_setpos] = sim.simxSetObjectPosition(clientID, TargetDummyHandle, rob_handle, target_dummy_pos, sim.simx_opmode_oneshot);
            % % % % %             [res_target_setorien] = sim.simxSetObjectOrientation(clientID, TargetDummyHandle, rob_handle, target_dummy_orien, sim.simx_opmode_oneshot);
            
            
            % % % % %             % % %             Change parent
            % % % % %             [res retPath retFloats retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
            % % % % %                 'ResizableFloor_5_25', ...
            % % % % %                 sim.sim_scripttype_childscript, ...
            % % % % %                 'change_parent', ...
            % % % % %                 [],[], ...
            % % % % %                 '', ...
            % % % % %                 [], ...
            % % % % %                 sim.simx_opmode_blocking);
            
            
            % % %                         Add to Collection
            % % % % %             [res_CreateCollec retInts_CreateCollec retFloats_CreateCollec retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
            % % % % %                 'ResizableFloor_5_25', ...
            % % % % %                 sim.sim_scripttype_childscript, ...
            % % % % %                 'addCollection', ...
            % % % % %                 [TargetDummyHandle],[], ...
            % % % % %                 '', ...
            % % % % %                 [], ...
            % % % % %                 sim.simx_opmode_blocking);
            
            
            
            %         getObjectPosition and getObjectOrientation must be executed twice to get the value, for unknown reasons
            % % %             [res_idlePos, idlePos] = sim.simxGetObjectPosition(clientID, TargetDummyHandle, -1, sim.simx_opmode_streaming);
            % % %             [res_idlePos, idleOrient] = sim.simxGetObjectOrientation(clientID, TargetDummyHandle, -1, sim.simx_opmode_streaming);
            % % %             idlePosOrient = [idlePos, idleOrient];
            
            
            %         disp(TargetDummyHandle);
            
            
            % % % % %             % %     Set link dummy
            % % % % %             [res_tip_dummy_handle, tip_dummy_handle] = sim.simxGetObjectHandle(clientID,'tip', sim.simx_opmode_blocking);
            % % % % %             [res_set_link_dummy, retPath, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
            % % % % %                 'ResizableFloor_5_25', ...
            % % % % %                 sim.sim_scripttype_childscript, ...
            % % % % %                 'set_link_dummy', ...
            % % % % %                 [tip_dummy_handle, TargetDummyHandle], ...
            % % % % %                 [], ...
            % % % % %                 '', ...
            % % % % %                 [], ...
            % % % % %                 sim.simx_opmode_blocking);
            
            % %     create IK group
            % % % % %             [res_create_IK, retPath, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
            % % % % %                 'ResizableFloor_5_25', ...
            % % % % %                 sim.sim_scripttype_childscript, ...
            % % % % %                 'create_IK_group', ...
            % % % % %                 [], ...
            % % % % %                 [], ...
            % % % % %                 '', ...
            % % % % %                 [], ...
            % % % % %                 sim.simx_opmode_blocking);
            %             pause(1);
            
            
            
            %                         % % % Create Collection manipulator
            %             [res_CreateCollec retInts_CreateCollec retFloats_CreateCollec retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
            %                 'ResizableFloor_5_25', ...
            %                 sim.sim_scripttype_childscript, ...
            %                 'createCollection_rob', ...
            %                 [rob_handle],[], ...
            %                 '', ...
            %                 [], ...
            %                 sim.simx_opmode_blocking);
            % %             pause(10);
            
            
            
            % % % % % %             Execute tasks     % % % % % %
            
            % % % Read the coordinates(position) of each device.
            % % %   [number returnCode,array position]=simxGetObjectPosition(number clientID,number objectHandle,number relativeToObjectHandle,number operationMode)
            %             [res_rob_handle, rob_handle] = sim.simxGetObjectHandle(clientID,'base_link_respondable',sim.simx_opmode_blocking)
            % % %             [res_rob_pos, rob_pos] = sim.simxGetObjectPosition(clientID, rob_handle, -1, sim.simx_opmode_streaming);
            % % %             %         disp(rob_pos);
            % % %
            % % %             [res_tab_pos, tab_pos] = sim.simxGetObjectPosition(clientID, tab_handle, -1, sim.simx_opmode_streaming);
            % % %
            % % %             [res_con_pos, con_pos] = sim.simxGetObjectPosition(clientID, con_handle, -1, sim.simx_opmode_streaming);
            % % %
            
            % % % % %             Generate Objects using CoppeliaSim functions % % % % %
            
            [res_con_getpos, con_pos] = sim.simxGetObjectPosition(clientID, con_handle, -1, sim.simx_opmode_blocking)
            % % % %     create rectangular on conveyor
            [res_cube_gen_0, retRectangularHandle, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                'ResizableFloor_5_25', ...
                sim.sim_scripttype_childscript, ...
                'createcube_function', ...
                [1,1,1], ... %   [color_flag(0=NULL,1=green),
                [con_pos(1), con_pos(2)+0.3, 0.9, 0.1, 0.1, 0.05], ...  %   [posX, posY, posZ, sizeX, sizeY, sizeZ]
                '', ...
                [], ...
                sim.simx_opmode_blocking);
            
            
            % % % %         disp(res_cube_gen_0);
            
            
            [res_tab_getpos, tab_pos_2] = sim.simxGetObjectPosition(clientID, tab_handle, -1, sim.simx_opmode_blocking)
            % % % %     create cube on table
            [res_cube_gen_1, retCubeHandle, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                'ResizableFloor_5_25', ...
                sim.sim_scripttype_childscript, ...
                'createcube_function', ...
                [0,1,1], ... %   [color_flag(0=NULL,1=green),
                [tab_pos_2(1), tab_pos_2(2), tab_pos_2(3)+0.1, 0.05, 0.05, 0.05], ...  %   [posX, posY, posZ, sizeX, sizeY, sizeZ]
                '', ...
                [], ...
                sim.simx_opmode_blocking);
            
            
            [res_rectangular_getpos, rectangular_pos] = sim.simxGetObjectPosition(clientID, retRectangularHandle(1), -1, sim.simx_opmode_blocking);
            
            [res_sensor_handle, Proximity_sensor_handle] = sim.simxGetObjectHandle(clientID, 'Proximity_sensor', sim.simx_opmode_blocking);
            [res_Psensor_getpos, Psensor_pos] = sim.simxGetObjectPosition(clientID, Proximity_sensor_handle, -1, sim.simx_opmode_blocking);

            
            
%             [res_con_getpos, con_pos] = sim.simxGetObjectPosition(clientID, con_handle, -1, sim.simx_opmode_oneshot);
            
            
            
            
            % let's define now the target positions needed
            fposition1 = [-0.36,    0.15,  0.75,    0,  0,  0];    % [x, y, z, alpha, beta, gamma] first position
            %         fposition2 = [0.2,    0,      0.9,    0,  0,  0];
            fposition3 = [con_pos(1), con_pos(2), 1.2,	0,	0,	0];    % above place position
                        fposition4 = [con_pos(1), con_pos(2), 1,	0,	0,	0];   % place position
            %             fposition3 = [0.625, 0.55, 1.2,	0,	0,	0];    % above place position
            %             fposition4 = [0.625, 0.55, 1,	0,	0,	0];   % place position
            
            fposition3 = [rectangular_pos(1), rectangular_pos(2), 1.2,	0,	0,	0];    % above place position
            fposition4 = [rectangular_pos(1), rectangular_pos(2), 1,	0,	0,	0];   % place position
            
            fposition3 = [Psensor_pos(1)-0.3, Psensor_pos(2), 1.2,	0,	0,	0];    % above place position
            fposition4 = [Psensor_pos(1)-0.3, Psensor_pos(2), 1,	0,	0,	0];   % place position
            
            fposition5 = [tab_pos_2(1),	tab_pos_2(2),   1.2,    0,  0,  0];    % above pickup position
            %         fposition5 = [tab_pos_2(1),	tab_pos_2(2),   tab_pos_2(3)+0.1,    0,  0,  0];    % above pickup position
            fposition6 = [tab_pos_2(1),	tab_pos_2(2),   tab_pos_2(3)+0.1,    0,  0,  0];    % pickup position
            
            % % %         Make fposition dummy
            [res_fpos3_DummyHandle, fpos3_DummyHandle] = sim.simxCreateDummy(clientID, 0.03, [], sim.simx_opmode_blocking);
            [res_fpos3_setpos] = sim.simxSetObjectPosition(clientID, fpos3_DummyHandle, -1, [fposition3(1), fposition3(2), fposition3(3)], sim.simx_opmode_oneshot);
            [res_fpos3_Dummy_setorien] = sim.simxSetObjectOrientation(clientID, fpos3_DummyHandle, rob_handle, target_dummy_orien, sim.simx_opmode_oneshot);
            
            [res_fpos4_DummyHandle, fpos4_DummyHandle] = sim.simxCreateDummy(clientID, 0.03, [], sim.simx_opmode_blocking);
            [res_fpos4_setpos] = sim.simxSetObjectPosition(clientID, fpos4_DummyHandle, -1, [fposition4(1), fposition4(2), fposition4(3)], sim.simx_opmode_oneshot);
            [res_fpos4_Dummy_setorien] = sim.simxSetObjectOrientation(clientID, fpos4_DummyHandle, rob_handle, target_dummy_orien, sim.simx_opmode_oneshot);
            
            [res_fpos5_DummyHandle, fpos5_DummyHandle] = sim.simxCreateDummy(clientID, 0.03, [], sim.simx_opmode_blocking);
            [res_fpos5_setpos] = sim.simxSetObjectPosition(clientID, fpos5_DummyHandle, -1, [fposition5(1), fposition5(2), fposition5(3)], sim.simx_opmode_oneshot);
            [res_fpos5_Dummy_setorien] = sim.simxSetObjectOrientation(clientID, fpos5_DummyHandle, rob_handle, target_dummy_orien, sim.simx_opmode_oneshot);
            
            [res_fpos6_DummyHandle, fpos6_DummyHandle] = sim.simxCreateDummy(clientID, 0.03, [], sim.simx_opmode_blocking);
            [res_fpos6_setpos] = sim.simxSetObjectPosition(clientID, fpos6_DummyHandle, -1, [fposition6(1), fposition6(2), fposition6(3)], sim.simx_opmode_oneshot);
            [res_fpos6_Dummy_setorien] = sim.simxSetObjectOrientation(clientID, fpos6_DummyHandle, rob_handle, target_dummy_orien, sim.simx_opmode_oneshot);
            
            % % %         Rename Dummy
            [res_time retInts_time retFloats_time retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
                'ResizableFloor_5_25', ...
                sim.sim_scripttype_childscript, ...
                'rename_object', ...
                [fpos3_DummyHandle, fpos4_DummyHandle, fpos5_DummyHandle, fpos6_DummyHandle],[], ...
                'fpos3_Dummy', ...
                [], ...
                sim.simx_opmode_blocking);
            
            
            % % % %             Read proximity sensor
%             [res_sensor_handle, Proximity_sensor_handle] = sim.simxGetObjectHandle(clientID, 'Proximity_sensor', sim.simx_opmode_blocking);
%             [res_read_sensor, detectionState, detectedPoint, detectedObjectHandle] = sim.simxReadProximitySensor(clientID, Proximity_sensor_handle, sim.simx_opmode_streaming);
            
            % % % %             disp(Proximity_sensor_handle);
            % % % %
            % % % %             disp('sensor state');
            % % % %
            % % % %             disp(res_read_sensor);
            % % % %             disp(detectionState);
            % % % %             disp(detectedPoint);
            % % % %             disp(detectedObjectHandle);
            % % % %             disp(cube0_handle);
            
            %              % % % %     Psensor
            %             [res_cube_gen_0, retInts, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
            %                 'ResizableFloor_5_25', ...
            %                 sim.sim_scripttype_childscript, ...
            %                 'read_Psensor', ...
            %                 [1,1,1], ... %   [color_flag(0=NULL,1=green),
            %                 [0, 0.9, 0.6, 0.1, 0.1, 0.05], ...  %   [posX, posY, posZ, sizeX, sizeY, sizeZ]
            %                 '', ...
            %                 [], ...
            %                 sim.simx_opmode_blocking);
            
%             
%             [res_cube0_pos, cube0_pos] = sim.simxGetObjectPosition(clientID, cube0_handle, -1, sim.simx_opmode_streaming);
%             
%             [res_cube1_handle, cube1_handle] = sim.simxGetObjectHandle(clientID,'Cuboid1', sim.simx_opmode_blocking);
%             
%             [res_cube0_pos, cube1_pos] = sim.simxGetObjectPosition(clientID, cube1_handle, -1, sim.simx_opmode_streaming);
            
            
            %         fposition4 = [cube0_pos(1), cube0_pos(2), cube0_pos(3)+0.05,	0,	0,	0]    % place position
            %         fposition3 = [cube0_pos(1), cube0_pos(2), cube0_pos(3)+0.1,	0,	0,	0]    % above place position
            
            
            % % %             % % % Create Collection
            % % %             [res_CreateCollec retInts_CreateCollec retFloats_CreateCollec retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
            % % %                 'ResizableFloor_5_25', ...
            % % %                 sim.sim_scripttype_childscript, ...
            % % %                 'createCollection', ...
            % % %                 [rob_handle, tab_handle, cube0_handle, cube1_handle, fpos3_DummyHandle, fpos4_DummyHandle, fpos5_DummyHandle, fpos6_DummyHandle],[], ...
            % % %                 '', ...
            % % %                 [], ...
            % % %                 sim.simx_opmode_blocking);
            % % %             %             pause(10);
            
            
            
            
            
            
            % % %             Execute pick and place  % % % % % % % % % % % %
            % Timer start
            % tic
            % % %         Get simtime
            [res_time retInts_time retFloats_time retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
                'ResizableFloor_5_25', ...
                sim.sim_scripttype_childscript, ...
                'get_simtime', ...
                [],[], ...
                '', ...
                [], ...
                sim.simx_opmode_blocking);
            
            %                         exeTime_tic = tic;
            %             disp(retFloats_time(1));
            
            
            
            
            
            
            % % %         Motion planning
            % %             [res_MP retInts_ retFloats_ retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
            % %                 'ResizableFloor_5_25', ...
            % %                 sim.sim_scripttype_childscript, ...
            % %                 'motionPlanning', ...
            % %                 [rob_handle],[], ...
            % %                 '', ...
            % %                 [], ...
            % %                 sim.simx_opmode_blocking);
            
            
            % % %             Get Target Dummies handle
            [res_Dum1_handle, Dum0_handle] = sim.simxGetObjectHandle(clientID, 'Dummy0', sim.simx_opmode_blocking);
            [res_Dum1_handle, Dum1_handle] = sim.simxGetObjectHandle(clientID, 'Dummy1', sim.simx_opmode_blocking);
            [res_Dum1_handle, Dum2_handle] = sim.simxGetObjectHandle(clientID, 'Dummy2', sim.simx_opmode_blocking);
            [res_Dum1_handle, Dum3_handle] = sim.simxGetObjectHandle(clientID, 'Dummy3', sim.simx_opmode_blocking);
            [res_Dum1_handle, Dum4_handle] = sim.simxGetObjectHandle(clientID, 'Dummy4', sim.simx_opmode_blocking);
            [res_Dum1_handle, Dum5_handle] = sim.simxGetObjectHandle(clientID, 'Dummy5', sim.simx_opmode_blocking);
            [res_Dum1_handle, Dum6_handle] = sim.simxGetObjectHandle(clientID, 'Dummy6', sim.simx_opmode_blocking);
            
            
            
            
            
            
            
            % % %         Motion planning
            
            % % %               Retrieve the poses (i.e. transformation matrices, 12 values, last row is implicit) of some dummies in the scene
            % % % % %             [res, retInts, TargetDummyPose, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
            % % % % %                 'remoteApiCommandServer', ...
            % % % % %                 sim.sim_scripttype_childscript, ...
            % % % % %                 'getObjectPose', ...
            % % % % %                 [TargetDummyHandle],[], ...
            % % % % %                 '', ...
            % % % % %                 [], ...
            % % % % %                 sim.simx_opmode_oneshot_wait);
            % % %               Retrieve the poses (i.e. transformation matrices, 12 values, last row is implicit) of some dummies in the scene
            [res, retInts, fpos3_DummyPose, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                'remoteApiCommandServer', ...
                sim.sim_scripttype_childscript, ...
                'getObjectPose', ...
                [fpos3_DummyHandle],[], ...
                '', ...
                [], ...
                sim.simx_opmode_oneshot_wait);
            % % %               Retrieve the poses (i.e. transformation matrices, 12 values, last row is implicit) of some dummies in the scene
            [res, retInts, fpos4_DummyPose, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                'remoteApiCommandServer', ...
                sim.sim_scripttype_childscript, ...
                'getObjectPose', ...
                [fpos4_DummyHandle],[], ...
                '', ...
                [], ...
                sim.simx_opmode_oneshot_wait);
            % % %               Retrieve the poses (i.e. transformation matrices, 12 values, last row is implicit) of some dummies in the scene
            [res, retInts, fpos5_DummyPose, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                'remoteApiCommandServer', ...
                sim.sim_scripttype_childscript, ...
                'getObjectPose', ...
                [fpos5_DummyHandle],[], ...
                '', ...
                [], ...
                sim.simx_opmode_oneshot_wait);
            % % %               Retrieve the poses (i.e. transformation matrices, 12 values, last row is implicit) of some dummies in the scene
            [res, retInts, fpos6_DummyPose, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                'remoteApiCommandServer', ...
                sim.sim_scripttype_childscript, ...
                'getObjectPose', ...
                [fpos6_DummyHandle],[], ...
                '', ...
                [], ...
                sim.simx_opmode_oneshot_wait);
            
            
            
            
            % % %     Get the robot initial state:
            [res, retInts, robotInitialState, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                'remoteApiCommandServer', ...
                sim.sim_scripttype_childscript, ...
                'getRobotState', ...
                [rob_handle],[], ...
                '', ...
                [], ...
                sim.simx_opmode_oneshot_wait);
            
            
            % % %     Some parameters:
            % % % % %             approachVector = [0,0,1];      % often a linear approach is required. This should also be part of the calculations when selecting an appropriate state for a given pose
            approachVector = [0,0,0.1];      % often a linear approach is required. This should also be part of the calculations when selecting an appropriate state for a given pose
            approachVector_0 = [0,0,0];      % often a linear approach is required. This should also be part of the calculations when selecting an appropriate state for a given pose
            
            maxConfigsForDesiredPose = 10; % we will try to find 10 different states corresponding to the goal pose and order them according to distance from initial state
            maxTrialsForConfigSearch = 300;    % a parameter needed for finding appropriate goal states
            searchCount = 1;       % how many times OMPL will run for a given task
            minConfigsForPathPlanningPath = 200;   % interpolation states for the OMPL path
            minConfigsForIkPath = 100; % interpolation states for the linear approach path
            collisionChecking = 1; % whether collision checking is on or off
            
            
            
            pathTime_tic = tic;
            
            % % %          Do the path planning here (between a start state and a goal pose, including a linear approach phase):
            inInts = [rob_handle, collisionChecking, minConfigsForIkPath, minConfigsForPathPlanningPath, maxConfigsForDesiredPose, maxTrialsForConfigSearch, searchCount]
            inFloats = horzcat(robotInitialState, fpos5_DummyPose, approachVector_0);
            %             inFloats = horzcat(robotInitialState, target3Pose, target1Pose);
            
            %                         inFloats = horzcat(robotInitialState, TargetDummyPose);
            
            %     res,retInts,path,retStrings,retBuffer=sim.simxCallScriptFunction(clientID,'remoteApiCommandServer',sim.sim_scripttype_childscript,'findPath_goalIsPose',inInts,inFloats,[],emptyBuff,sim.simx_opmode_oneshot_wait)
            
            [res retInts path retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
                'remoteApiCommandServer', ...
                sim.sim_scripttype_childscript, ...
                'findPath_goalIsPose', ...
                inInts,inFloats, ...
                '', ...
                [], ...
                sim.simx_opmode_oneshot_wait);
            
            if (res==0) && length(path)>0
                % % %          The path could be in 2 parts: a path planning path, and a linear approach path:
                part1StateCnt = retInts(1);
                part2StateCnt = retInts(2);
                path1 = path(1:part1StateCnt*6);
                
                
                % % %                Visualize the first path:
                [res retInts retFloats retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
                    'remoteApiCommandServer', ...
                    sim.sim_scripttype_childscript, ...
                    'visualizePath', ...
                    [rob_handle,255,0,255],path, ...
                    '', ...
                    [], ...
                    sim.simx_opmode_oneshot_wait);
                
                line1Handle = retInts(1);
                
                % % %          Make the robot follow the path:
                [res retInts retFloats retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
                    'remoteApiCommandServer', ...
                    sim.sim_scripttype_childscript, ...
                    'runThroughPath', ...
                    [rob_handle],path, ...
                    '', ...
                    [], ...
                    sim.simx_opmode_oneshot_wait);
                
                exeTime_pahtT = toc(pathTime_tic)
                
                
                % % %              Wait until the end of the movement:
                runningPath = true;
                while runningPath
                    %                     res,retInts,retFloats,retStrings,retBuffer=sim.simxCallScriptFunction(clientID,'remoteApiCommandServer',sim.sim_scripttype_childscript,'isRunningThroughPath',[robotHandle],[],[],emptyBuff,sim.simx_opmode_oneshot_wait)
                    
                    [res retInts retFloats retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
                        'remoteApiCommandServer', ...
                        sim.sim_scripttype_childscript, ...
                        'isRunningThroughPath', ...
                        [rob_handle],[], ...
                        '', ...
                        [], ...
                        sim.simx_opmode_oneshot_wait);
                    
                    runningPath = retInts(1) == 1;
                end
                
                path2 = path(part1StateCnt*6:end);
                
                % % %             Suck objects
                [res retPath retFloats retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
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
                
                
                % % %          Do the path planning here (between a start state and a goal pose, including a linear approach phase):
                approachVector2 = [0,0,-0.1];
                
                inInts = [rob_handle, collisionChecking, minConfigsForIkPath, minConfigsForPathPlanningPath, maxConfigsForDesiredPose, maxTrialsForConfigSearch, searchCount]
                inFloats = horzcat(robotCurrentConfig, fpos6_DummyPose, approachVector_0);
                % inFloats = horzcat(robotCurrentConfig, target3Pose, target1Pose);
                
                
                [res, retInts, path, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                    'remoteApiCommandServer', ...
                    sim.sim_scripttype_childscript, ...
                    'findPath_goalIsPose', ...
                    inInts,inFloats, ...
                    '', ...
                    [], ...
                    sim.simx_opmode_oneshot_wait);
                
                if (res==0) && length(path)>0
                    
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
                    [res retPath retFloats retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
                        'IRB4600_connection', ...
                        sim.sim_scripttype_childscript, ...
                        'release_object', ...
                        [],[], ...
                        '', ...
                        [], ...
                        sim.simx_opmode_blocking);
                    
                end
                
                
                
                
                
                % % % % %
                % % % % %                                 % % %      Visualize the second path (the linear approach):
                % % % % %                                 %         res,retInts,retFloats,retStrings,retBuffer=sim.simxCallScriptFunction(clientID,'remoteApiCommandServer',sim.sim_scripttype_childscript,'visualizePath',[robotHandle,0,255,0],path2,[],emptyBuff,sim.simx_opmode_oneshot_wait)
                % % % % %
                % % % % %                                 [res retInts retFloats retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
                % % % % %                                     'remoteApiCommandServer', ...
                % % % % %                                     sim.sim_scripttype_childscript, ...
                % % % % %                                     'visualizePath', ...
                % % % % %                                     [rob_handle,0,255,0],path2, ...
                % % % % %                                     '', ...
                % % % % %                                     [], ...
                % % % % %                                     sim.simx_opmode_oneshot_wait);
                % % % % %
                % % % % %                                 line2Handle = retInts(1);
                % % % % %
                % % % % %                                 % % %          Make the robot follow the path:
                % % % % %                                 [res retInts retFloats retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
                % % % % %                                     'remoteApiCommandServer', ...
                % % % % %                                     sim.sim_scripttype_childscript, ...
                % % % % %                                     'runThroughPath', ...
                % % % % %                                     [rob_handle],path2, ...
                % % % % %                                     '', ...
                % % % % %                                     [], ...
                % % % % %                                     sim.simx_opmode_oneshot_wait);
                
                
                % % % % %                 % % %              Wait until the end of the movement:
                % % % % %                 runningPath = true;
                % % % % %                 while runningPath
                % % % % %                     %                     res,retInts,retFloats,retStrings,retBuffer=sim.simxCallScriptFunction(clientID,'remoteApiCommandServer',sim.sim_scripttype_childscript,'isRunningThroughPath',[robotHandle],[],[],emptyBuff,sim.simx_opmode_oneshot_wait)
                % % % % %
                % % % % %                     [res retInts retFloats retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
                % % % % %                         'remoteApiCommandServer', ...
                % % % % %                         sim.sim_scripttype_childscript, ...
                % % % % %                         'isRunningThroughPath', ...
                % % % % %                         [rob_handle],[], ...
                % % % % %                         '', ...
                % % % % %                         [], ...
                % % % % %                         sim.simx_opmode_oneshot_wait);
                % % % % %
                % % % % %                     runningPath = retInts(1) == 1;
                % % % % %                 end
                
                
                
                
                
            end
            %
            %              -- Get handles and postions of dummies
            
            %     idlePos = sim.getObjectPosition(targetDummy,-1)
            %     idleOrient = sim.getObjectOrientation(targetDummy,-1)
            %
            %     releasePosHandle = sim.getObjectHandle("releasePos")
            %     releasePos = sim.getObjectPosition(releasePosHandle,-1)
            %     releaseOrient = sim.getObjectOrientation(releasePosHandle,-1)+
            %                 releasePath = createPath("releasePath",idlePos,idleOrient,releasePos,releaseOrient)
            %
            
            
            %         [res_targetDummy_handle, targetDummy_handle] = sim.simxGetObjectHandle(clientID,'Dummy', sim.simx_opmode_blocking);
            
            % % %             [res_idlePos, idlePos] = sim.simxGetObjectPosition(clientID, TargetDummyHandle, -1, sim.simx_opmode_streaming);
            % % %             [res_idlePos, idleOrient] = sim.simxGetObjectOrientation(clientID, TargetDummyHandle, -1, sim.simx_opmode_streaming);
            % % %             idlePosOrient = [idlePos, idleOrient];
            
            %                 [res_fpos5_DummyHandle, fpos5_DummyHandle] = sim.simxGetObjectHandle(clientID,'Dummy', sim.simx_opmode_blocking);
            
            % % % % %             % % %         Create Path initialPos to fpos5
            % % % % %             [res retPathI5 retFloats retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
            % % % % %                 'ResizableFloor_5_25', ...
            % % % % %                 sim.sim_scripttype_childscript, ...
            % % % % %                 'createPath', ...
            % % % % %                 [],[idlePosOrient, fposition5], ...
            % % % % %                 'path_fposInit_to_5', ...
            % % % % %                 [], ...
            % % % % %                 sim.simx_opmode_blocking);
            % % % % %
            % % % % %
            % % % % %             % % %         Create Path fpos5 to fpos6
            % % % % %             [res retPath56 retFloats retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
            % % % % %                 'ResizableFloor_5_25', ...
            % % % % %                 sim.sim_scripttype_childscript, ...
            % % % % %                 'createPath', ...
            % % % % %                 [],[fposition5, fposition6], ...
            % % % % %                 'path_fpos5_to_6', ...
            % % % % %                 [], ...
            % % % % %                 sim.simx_opmode_blocking);
            % % % % %             %         disp(retPath56(1));
            % % % % %
            % % % % %             % % %         Create Path fpos5 to fpos3
            % % % % %             [res retPath53 retFloats retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
            % % % % %                 'ResizableFloor_5_25', ...
            % % % % %                 sim.sim_scripttype_childscript, ...
            % % % % %                 'createPath', ...
            % % % % %                 [],[fposition5, fposition3], ...
            % % % % %                 'path_fpos5_to_3', ...
            % % % % %                 [], ...
            % % % % %                 sim.simx_opmode_blocking);
            % % % % %
            % % % % %             % % %         Create Path fpos5 to fpos3
            % % % % %             [res retPath34 retFloats retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
            % % % % %                 'ResizableFloor_5_25', ...
            % % % % %                 sim.sim_scripttype_childscript, ...
            % % % % %                 'createPath', ...
            % % % % %                 [],[fposition3, fposition4], ...
            % % % % %                 'path_fpos3_to_4', ...
            % % % % %                 [], ...
            % % % % %                 sim.simx_opmode_blocking);
            % % % % %
            % % % % %             % % %         Following path
            % % % % %             % % %         Follow Path fpos5 to fpos6
            % % % % %             [res retFollowPath56 retFloats retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
            % % % % %                 'ResizableFloor_5_25', ...
            % % % % %                 sim.sim_scripttype_childscript, ...
            % % % % %                 'followPath', ...
            % % % % %                 [TargetDummyHandle, retPath56(1)],[fposition5, fposition6], ...
            % % % % %                 '', ...
            % % % % %                 [], ...
            % % % % %                 sim.simx_opmode_blocking);
            % % % % %
            % % % % %             disp(TargetDummyHandle);
            % % % % %             disp(retPath56(1));
            
            %         moveL (clientID, TargetDummyHandle, fposition5, 8);
            %         moveL (clientID, TargetDummyHandle, fposition6, 8);
            
            % % %             Suck objects
            % % % % %             [res retPath retFloats retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
            % % % % %                 'suctionPad', ...
            % % % % %                 sim.sim_scripttype_childscript, ...
            % % % % %                 'suck_object', ...
            % % % % %                 [],[], ...
            % % % % %                 '', ...
            % % % % %                 [], ...
            % % % % %                 sim.simx_opmode_blocking);
            % % % % %
            % % % % %             %         moveL (clientID, TargetDummyHandle, fposition5, 8);
            % % % % %
            % % % % %             %         moveL (clientID, TargetDummyHandle, fposition3, 8);
            % % % % %             %         moveL (clientID, TargetDummyHandle, fposition4, 8);
            % % % % %
            % % % % %             [res retPath retFloats retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
            % % % % %                 'suctionPad', ...
            % % % % %                 sim.sim_scripttype_childscript, ...
            % % % % %                 'release_object', ...
            % % % % %                 [],[], ...
            % % % % %                 '', ...
            % % % % %                 [], ...
            % % % % %                 sim.simx_opmode_blocking);
            
            %         moveL (clientID, TargetDummyHandle, fposition3, 8);
            
            % % %         Get simtime
            [res_time2 retInts_time2 retFloats_time2 retStrings retBuffer] = sim.simxCallScriptFunction(clientID, ...
                'ResizableFloor_5_25', ...
                sim.sim_scripttype_childscript, ...
                'get_simtime', ...
                [],[], ...
                '', ...
                [], ...
                sim.simx_opmode_blocking);
            
            % % % %             disp(retFloats_time2);
            
            simTime = retFloats_time2 - retFloats_time;
            %                         exeTime_T = toc(exeTime_tic);
            
            
            % % %             % % %         Save the relationship between "repetition rate" and "simTime" to csv
            % % %             simTime_array(1, 1) = rep_rate;
            % % %             simTime_array(1, 2) = simTime;
            % % %
            % % %             %         writematrix(simTime_array_cap,'simTime_array.csv');
            % % %             %         writematrix(simTime_array,'simTime_array.csv','WriteMode','append');
            % % %
            % % %             fileID = fopen(simTime_file_name_csv);
            % % %             dlmwrite(simTime_file_name_csv, simTime_array,'-append');
            % % %             fclose(fileID);
            
            
            %             simTime_array(count,1) = simTime;
            % % % %             disp(simTime_array);
            
            % % %             % % % % %             Update Minimum exetime
            % % %             if rep_rate == 1   %% if first time
            % % %                 disp('UPDATE min_exetime');
            % % %                 repetition_rate = rep_rate;
            % % %                 min_simTime = simTime;
            % % %                 optim_tabpos = tab_pos_2;
            % % %                 optim_robpos = rob_pos_2;
            % % %
            % % %             elseif simTime < min_simTime
            % % %                 disp('UPDATE min_exetime');
            % % %                 repetition_rate = rep_rate;
            % % %                 min_simTime = simTime;
            % % %                 optim_tabpos = tab_pos_2;
            % % %                 optim_robpos = rob_pos_2;
            % % %
            % % %             else
            % % %             end
            
            % Timer stop
            % elapsedTime = toc
            
            
            %{
        if(detectionState < 0)
            
        else
            
            % % %             Remove a cube
            [res_cube0_remove] = sim.simxRemoveObject(clientID, cube0_handle, sim.simx_opmode_blocking);
            [res_cube1_remove] = sim.simxRemoveObject(clientID, cube1_handle, sim.simx_opmode_blocking);
            
            % % % % %             Generate Objects using CoppeliaSim functions % % % % %
            % % % %     create rectangular on conveyor
            [res_cube_gen_0, retInts, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                'ResizableFloor_5_25', ...
                sim.sim_scripttype_childscript, ...
                'createcube_function', ...
                [1,1,1], ... %   [color_flag(0=NULL,1=green),
                [0, 0.9, 0.6, 0.1, 0.1, 0.05], ...  %   [posX, posY, posZ, sizeX, sizeY, sizeZ]
                '', ...
                [], ...
                sim.simx_opmode_blocking);
            
            disp('ressssssssssss');
            disp(res_cube_gen_0);
            
            % % % %     create cube on table
            [res_cube_gen_1, retInts, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                'ResizableFloor_5_25', ...
                sim.sim_scripttype_childscript, ...
                'createcube_function', ...
                [0,1,1], ... %   [color_flag(0=NULL,1=green),
                [-0.6 0.65 0.6, 0.05, 0.05, 0.05], ...  %   [posX, posY, posZ, sizeX, sizeY, sizeZ]
                '', ...
                [], ...
                sim.simx_opmode_blocking);
            
            [res_cube0_handle, cube0_handle] = sim.simxGetObjectHandle(clientID,'Cuboid0', sim.simx_opmode_blocking);
            [res_cube1_handle, cube1_handle] = sim.simxGetObjectHandle(clientID,'Cuboid1', sim.simx_opmode_blocking);
            %           [res_cube2_handle, cube2_handle] = sim.simxGetObjectHandle(clientID,'Cuboid2', sim.simx_opmode_blocking)
            
            
        end
            %}
            
            
            
            %         pause(1);
            %         %{
            
            
            % % %         Remove dummy
            [res_fpos3_DummyHandle_remove] = sim.simxRemoveObject(clientID, fpos3_DummyHandle, sim.simx_opmode_blocking);
            [res_fpos4_DummyHandle_remove] = sim.simxRemoveObject(clientID, fpos4_DummyHandle, sim.simx_opmode_blocking);
            [res_fpos5_DummyHandle_remove] = sim.simxRemoveObject(clientID, fpos5_DummyHandle, sim.simx_opmode_blocking);
            [res_fpos6_DummyHandle_remove] = sim.simxRemoveObject(clientID, fpos6_DummyHandle, sim.simx_opmode_blocking);
            
            
            
            % % % % %             % % %         Remove Path
            % % % % %             [res_PathI5_remove] = sim.simxRemoveObject(clientID, retPathI5(1), sim.simx_opmode_blocking);
            % % % % %             [res_Path56_remove] = sim.simxRemoveObject(clientID, retPath56(1), sim.simx_opmode_blocking);
            % % % % %             [res_Path53_remove] = sim.simxRemoveObject(clientID, retPath53(1), sim.simx_opmode_blocking);
            % % % % %             [res_Path34_remove] = sim.simxRemoveObject(clientID, retPath34(1), sim.simx_opmode_blocking);
            
            
            
            % % %             Remove a TargetDummy
            % % % % %             [res_targetdummy_remove] = sim.simxRemoveObject(clientID, TargetDummyHandle, sim.simx_opmode_blocking);
            
            % % %             Remove cubes
            [res_Rectangular_remove] = sim.simxRemoveObject(clientID, retRectangularHandle, sim.simx_opmode_blocking);
            [res_Cube_remove] = sim.simxRemoveObject(clientID, retCubeHandle, sim.simx_opmode_blocking);
            
            
            % % %             Remove model (Matlab function)  % % % % %
            % % %             Remove a robot
            [res_rob_remove] = sim.simxRemoveModel(clientID, rob_handle, sim.simx_opmode_blocking);
            % % %             Remove a conveyor
            %             [res_con_remove] = sim.simxRemoveModel(clientID, con_handle, sim.simx_opmode_blocking);
            % % %             Remove a table
            [res_con_remove] = sim.simxRemoveModel(clientID, tab_handle, sim.simx_opmode_blocking);
            
            % % %         Disable logging
            diary off
            %         csvwrite('simTime_array.csv', simTime_array);
            
            % fileID = fopen('simTime_array.csv','w');
            % % fprintf(fileID,'%6s %12s\n','x','exp(x)');
            % fprintf(fileID,'%6.2f %12.8f\n',A);
            % fclose(fileID);
            
            exeTime_T = toc(exeTime_tic);
            
            % % %         Save the relationship between "repetition rate" and "exeTime" to csv
            % % %             exeTime_array(1, 1) = rep_rate;
            % % %             exeTime_array(1, 2) = exeTime_T;
            % % %
            % % %             fileID = fopen(exeTime_file_name_csv);
            % % %             dlmwrite(exeTime_file_name_csv, exeTime_array,'-append');
            % % %             fclose(fileID);
            
            
            
        end        % % %     Repetition in "rep times" units
        
        TimeToRestart_tic = tic;
        
        % % %         % % % Restart simulation
        % % %         [res_sim_stop] = sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
        % % %         is_running = true;
        % % %
        % % %         % Wait until the simulation is completely stopped
        % % %         while is_running
        % % %             [res_GetPing, ping_time] = sim.simxGetPingTime(clientID);
        % % %             [res_GetInMes, server_state] = sim.simxGetInMessageInfo(clientID, sim.simx_headeroffset_server_state);
        % % %             is_running = bitand(server_state,1);
        % % %         end
        % % %         %         pause(5);
        % % %
        % % %         % % %         Do not StartSimulation on the last attempt
        % % %         if (rep_rate == rep*rep_overall)
        % % %         else
        % % %             [res_startSim] = sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)
        % % %         end
        % % %
        % % %         TimeToRestart_T = toc(TimeToRestart_tic);
        % % %
        % % %         % % %         Save the relationship between "repetition rate" and "TimeToRestart" to csv
        % % %         TimeToRestart_array(1, 1) = count_overall;
        % % %         TimeToRestart_array(1, 2) = TimeToRestart_T;
        % % %
        % % %         fileID = fopen(TimeToRestart_file_name_csv);
        % % %         dlmwrite(TimeToRestart_file_name_csv, TimeToRestart_array,'-append');
        % % %         fclose(fileID);
        % % %
        
        
    end            % % %     Repetition overall
    
    
    
    %     simulation pause
    %     [res_sim_pause] = sim.simxPauseSimulation(clientID, sim.simx_opmode_oneshot);
    %     pause(5);
    
    
    [res_con_remove] = sim.simxRemoveModel(clientID, con_handle, sim.simx_opmode_blocking);
    
    fprintf('Optimized Table position:\n');
    disp(optim_tabpos);
    fprintf('Optimized Manipulator position:\n');
    disp(optim_robpos);
    %     fprintf('Optimized Manipulator position: %f\n', optim_robpos);
    fprintf('Repetition Rate when Min Execute Time :%d\n', repetition_rate);
    fprintf('Min Execute Time(sim time)[s] :%f\n', min_simTime);
    %     disp(simTime_array);
    
    % Timer stop[s]
    elapsedTime = toc(tStart);
    
    fprintf('elapsedTime(real time)[s] :%f\n', elapsedTime);
    
    
    if (res_rob_genetate == sim.simx_return_ok)
        %             fprintf('Returned message: %s\n',retStrings);
    else
        fprintf('Remote function call failed\n');
    end
    
    
    % Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID);
    
else
    disp('Failed connecting to remote API server');
end
sim.delete(); % call the destructor!

disp('Program ended');

% % Timer stop[s]
% elapsedTime = toc(tStart)

% whos

end
