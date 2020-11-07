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
tStart = tic


disp('Program started');
% sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
sim = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1',19999,true,true,5000,5);



if (clientID>-1)
    disp('Connected to remote API server');
    
    num_t = 1;
    
    % % %             Genarate a conveyor
    [res_con_genetate, con_handle] = sim.simxLoadModel(clientID,'customizable_conveyor_belt_03x1x05_fix_sensorPos.ttm', 0, sim.simx_opmode_blocking);
    [res_con_setpos] = sim.simxSetObjectPosition(clientID, con_handle, -1, [0 0.5 0.45], sim.simx_opmode_oneshot);
    [res_con_setorien] = sim.simxSetObjectOrientation(clientID, con_handle, -1, [0 0 -pi/2], sim.simx_opmode_oneshot);
    
    % % %     number of repetition
    rep = 100;
    
    % % %         Repeated creation and deletion of the device.
    for count = 1:rep
        
        % % % % %         Display the number of repetitions in CoppeliaSim
        [res_print_repetition_rate, retInts, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
            'ResizableFloor_5_25', ...
            sim.sim_scripttype_childscript, ...
            'print_repetition_rate', ...
            [num_t], ...
            [], ...
            '', ...
            [], ...
            sim.simx_opmode_blocking);
        
        % % % % %         Random search to locate manipulators and tables.
        [m_pos_random, p_pos_random, c_pos_random] = random_research_forum();
        rob_pos_2 = [m_pos_random(1), m_pos_random(2), 0.15];
        tab_pos_2 = [p_pos_random(1), p_pos_random(2), 0.45];
        
        
        % % % % %             Generate facilities using MATLAB functions % % % % %
        % % %             Genarate a robot
        [res_rob_genetate, rob_handle] = sim.simxLoadModel(clientID,'motoman_HP3J_with_base_JointsLimit2.ttm', 0, sim.simx_opmode_blocking);
        [res_rob_setpos] = sim.simxSetObjectPosition(clientID, rob_handle, -1, rob_pos_2, sim.simx_opmode_oneshot);
        
        % % %             Setting of target dummy
        [res_target_gen, TargetDummyHandle] = sim.simxCreateDummy(clientID, 0.03, [], sim.simx_opmode_blocking);
        [res_target_setpos] = sim.simxSetObjectPosition(clientID, TargetDummyHandle, rob_handle, [0.25, 0, 0.4], sim.simx_opmode_oneshot);
        
        % %     Set link dummy
        [res_tip_dummy_handle, tip_dummy_handle] = sim.simxGetObjectHandle(clientID,'tip', sim.simx_opmode_blocking);
        [res_set_link_dummy, retInts, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
            'ResizableFloor_5_25', ...
            sim.sim_scripttype_childscript, ...
            'set_link_dummy', ...
            [tip_dummy_handle, TargetDummyHandle], ...
            [], ...
            '', ...
            [], ...
            sim.simx_opmode_blocking);
        
        % %     create IK group
        [res_create_IK, retInts, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
            'ResizableFloor_5_25', ...
            sim.sim_scripttype_childscript, ...
            'create_IK_group', ...
            [], ...
            [], ...
            '', ...
            [], ...
            sim.simx_opmode_blocking);
        
        
        % % %             Genarate a table
        [res_tab_genetate, tab_handle] = sim.simxLoadModel(clientID,'customizable_table_with_create_cube_func.ttm', 0, sim.simx_opmode_blocking);
        [res_tab_setpos] = sim.simxSetObjectPosition(clientID, tab_handle, -1, tab_pos_2, sim.simx_opmode_oneshot);
        
        
        % % % % % %             Execute Pick and Place     % % % % % %
        % % % Read the coordinates(position) of each facility.
        [res_rob_pos, rob_pos] = sim.simxGetObjectPosition(clientID, rob_handle, -1, sim.simx_opmode_streaming);
        [res_tab_pos, tab_pos] = sim.simxGetObjectPosition(clientID, tab_handle, -1, sim.simx_opmode_streaming);
        [res_con_pos, con_pos] = sim.simxGetObjectPosition(clientID, con_handle, -1, sim.simx_opmode_streaming);
        
        
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
        
        % % % %     create cube on table
        [res_cube_gen_1, retInts, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
            'ResizableFloor_5_25', ...
            sim.sim_scripttype_childscript, ...
            'createcube_function', ...
            [0,1,1], ... %   [color_flag(0=NULL,1=green),
            [tab_pos_2(1), tab_pos_2(2), tab_pos_2(3)+0.1, 0.05, 0.05, 0.05], ...  %   [posX, posY, posZ, sizeX, sizeY, sizeZ]
            '', ...
            [], ...
            sim.simx_opmode_blocking);
        
        
        [res_cube0_handle, cube0_handle] = sim.simxGetObjectHandle(clientID,'Cuboid0', sim.simx_opmode_blocking);
        [res_cube1_handle, cube1_handle] = sim.simxGetObjectHandle(clientID,'Cuboid1', sim.simx_opmode_blocking);
        
        
        % let's define now the target positions needed
        fposition3 = [0, 0.52, 0.65,	0,	0,	0];    % above place position
        fposition4 = [0, 0.52, 0.625,	0,	0,	0];   % place position
        fposition5 = [tab_pos_2(1),	tab_pos_2(2),   0.65,    0,  0,  0];    % above pickup position
        fposition6 = [tab_pos_2(1),	tab_pos_2(2),   tab_pos_2(3)+0.13,    0,  0,  0];    % pickup position
        
        
        % Timer start
        % tic
        
        % % %         Get the simtime on coppeliasim
        [res_time, retInts_time, retFloats_time, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
            'ResizableFloor_5_25', ...
            sim.sim_scripttype_childscript, ...
            'get_simtime', ...
            [],[], ...
            '', ...
            [], ...
            sim.simx_opmode_blocking);
        
        % % %             Move the end effector to position 5.
        moveL (clientID, TargetDummyHandle, fposition5, 8);
        % % %             Move the end effector to position 6.
        moveL (clientID, TargetDummyHandle, fposition6, 8);
        
        
        % % %             Suck a cube
        [res, retInts, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
            'suctionPad', ...
            sim.sim_scripttype_childscript, ...
            'suck_object', ...
            [0,0,1],[0.1,0.3,0.68], ...
            '', ...
            [], ...
            sim.simx_opmode_blocking);
        
        % % %             Move the end effector to position 5.
        moveL (clientID, TargetDummyHandle, fposition5, 8);
        % % %             Move the end effector to position 3.
        moveL (clientID, TargetDummyHandle, fposition3, 8);
        % % %             Move the end effector to position 4.
        moveL (clientID, TargetDummyHandle, fposition4, 8);
        
        % % %             Release the cube.
        [res, retInts, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
            'suctionPad', ...
            sim.sim_scripttype_childscript, ...
            'release_object', ...
            [0,0,1],[0.1,0.3,0.68], ...
            '', ...
            [], ...
            sim.simx_opmode_blocking);
        
        % % %             Move the end effector to position 4.
        moveL (clientID, TargetDummyHandle, fposition3, 8);
        
        % % %         Get the simtime on coppeliasim
        [res_time2, retInts_time2, retFloats_time2, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
            'ResizableFloor_5_25', ...
            sim.sim_scripttype_childscript, ...
            'get_simtime', ...
            [],[], ...
            '', ...
            [], ...
            sim.simx_opmode_blocking);
        
        % % %             The time it took to perform the work
        exeTime = retFloats_time2 - retFloats_time
        
        
        % % %         Find the one with the shortest pick-and-place execution time among the generated placements.
        % % %             Update Minimum exetime
        if num_t == 1   %% if first time
            disp('UPDATE min_exetime');
            repetition_rate = num_t;
            min_exeTime = exeTime;
            optim_tabpos = tab_pos_2;
            optim_robpos = rob_pos_2;
            
        elseif exeTime < min_exeTime
            disp('UPDATE min_exetime');
            repetition_rate = num_t;
            min_exeTime = exeTime;
            optim_tabpos = tab_pos_2;
            optim_robpos = rob_pos_2;
            
        else
        end
        
        % % %             Remove a TargetDummy
        [res_targetdummy_remove] = sim.simxRemoveObject(clientID, TargetDummyHandle, sim.simx_opmode_blocking);
        
        % % %             Remove cubes
        [res_cube0_remove] = sim.simxRemoveObject(clientID, cube0_handle, sim.simx_opmode_blocking);
        [res_cube1_remove] = sim.simxRemoveObject(clientID, cube1_handle, sim.simx_opmode_blocking);
        
        
        % % %             Remove model (Matlab function)  % % % % %
        % % %             Remove a robot
        [res_rob_remove] = sim.simxRemoveModel(clientID, rob_handle, sim.simx_opmode_blocking);
        % % %             Remove a table
        [res_con_remove] = sim.simxRemoveModel(clientID, tab_handle, sim.simx_opmode_blocking);
        %         %}
        
        num_t = num_t + 1;
    end
    
    %     simulation pause
    [res_sim_pause] = sim.simxPauseSimulation(clientID, sim.simx_opmode_oneshot);
    
    % % %             Remove a conveyor
    [res_con_remove] = sim.simxRemoveModel(clientID, con_handle, sim.simx_opmode_blocking);
    
    fprintf('Optimized Table position:\n');
    disp(optim_tabpos);
    fprintf('Optimized Manipulator position:\n');
    disp(optim_robpos);
    %     fprintf('Optimized Manipulator position: %f\n', optim_robpos);
    fprintf('Repetition Rate when Min Execute Time :%d\n', repetition_rate);
    fprintf('Min Execute Time(sim time)[s] :%f\n', min_exeTime);
    
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

% whos

end
