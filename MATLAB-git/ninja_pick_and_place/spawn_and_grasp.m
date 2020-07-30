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

    disp('Program started');
    % sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    sim = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections
    clientID = sim.simxStart('127.0.0.1',19999,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');

        % % %         Repeated creation and deletion of the device.
        for count = 1:1


            % % % % %             Generate facilities using MATLAB functions % % % % %
            % % %             Genarate a robot
            [res_rob_genetate, rob_handle] = sim.simxLoadModel(clientID,'motoman_HP3J_with_base_2.ttm', 0, sim.simx_opmode_blocking);
%             [res_rob_genetate, rob_handle] = sim.simxLoadModel(clientID,'motoman_HP3J_with_base_and_targetDummy.ttm', 0, sim.simx_opmode_blocking);
            [res_rob_setpos] = sim.simxSetObjectPosition(clientID, rob_handle, -1, [-0.6 0.15 0.15], sim.simx_opmode_oneshot);
            
            %{
            % % %             Setting of target dummy (If generate target dummy with robot)
            [res_target_handle, target_handle] = sim.simxGetObjectHandle(clientID,'target', sim.simx_opmode_blocking)
%             [res_target_handle2, target_handle2] = sim.simxGetObjectSelection(clientID, sim.simx_opmode_blocking)   % What handle are you getting?
            [res_target_parent] = sim.simxSetObjectParent(clientID, target_handle, -1, 0, sim.simx_opmode_blocking)
            
%             [res_target_pos, target_pos] = sim.simxGetObjectPosition(clientID, res_target_handle, -1, sim.simx_opmode_streaming)
            [res_target_setpos] = sim.simxSetObjectPosition(clientID, target_handle, -1, [-0.36 0.15 0.75], sim.simx_opmode_oneshot);
            [res_target_setorien] = sim.simxSetObjectOrientation(clientID, target_handle, -1, [0 0 -pi], sim.simx_opmode_oneshot)
            %}
            
            % % %             Setting of target dummy (If generate only target dummy)
            [res_target_gen, dummyHandle] = sim.simxCreateDummy(clientID, 0.03, [], sim.simx_opmode_blocking)
            [res_target_setpos] = sim.simxSetObjectPosition(clientID, dummyHandle, -1, [-0.36 0.15 0.75], sim.simx_opmode_oneshot);
%             [res_target_setorien] = sim.simxSetObjectOrientation(clientID, dummyHandle, -1, [0 0 -pi], sim.simx_opmode_oneshot)
            
            % %     Set link dummy
            [res_tip_dummy_handle, tip_dummy_handle] = sim.simxGetObjectHandle(clientID,'tip', sim.simx_opmode_blocking)
            [res_set_link_dummy, retInts, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                'ResizableFloor_5_25', ...
                sim.sim_scripttype_childscript, ...
                'set_link_dummy', ...
                [tip_dummy_handle, dummyHandle], ... 
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



            
            % % %             Genarate a conveyor
            [res_con_genetate, con_handle] = sim.simxLoadModel(clientID,'customizable conveyor belt_03x1x05.ttm', 0, sim.simx_opmode_blocking);
            [res_con_setpos] = sim.simxSetObjectPosition(clientID, con_handle, -1, [0 0.5 0.45], sim.simx_opmode_oneshot);
            [res_con_setorien] = sim.simxSetObjectOrientation(clientID, con_handle, -1, [0 0 -pi/2], sim.simx_opmode_oneshot)

            % % %             Genarate a table
            [res_tab_genetate, tab_handle] = sim.simxLoadModel(clientID,'customizable_table_with_create_cube_func.ttm', 0, sim.simx_opmode_blocking);
            [res_tab_setpos] = sim.simxSetObjectPosition(clientID, tab_handle, -1, [-0.6 0.65 0.45], sim.simx_opmode_oneshot);

%             pause(5);


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

            disp(retInts);
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

            [res_cube0_handle, cube0_handle] = sim.simxGetObjectHandle(clientID,'Cuboid0', sim.simx_opmode_blocking)
            [res_cube1_handle, cube1_handle] = sim.simxGetObjectHandle(clientID,'Cuboid1', sim.simx_opmode_blocking)
            %           [res_cube2_handle, cube2_handle] = sim.simxGetObjectHandle(clientID,'Cuboid2', sim.simx_opmode_blocking)


            %{
    % % % %     Generate Assembly example
    % % % %     create rectangular on conveyor
     [res, retInts, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                    'ResizableFloor_5_25', ...
                    sim.sim_scripttype_childscript, ...
                    'createcube_function', ...
                    [1,1,1], ... %   [color_flag(0=NULL,1=green),
                    [1, 1, 0.3, 0.1, 0.1, 0.05], ...  %   [posx, posy, posz, sizex, sizey, sizez]
                    '', ...
                    [], ...
                    sim.simx_opmode_blocking);

                disp(retInts);

    % % % %     create cube on table
     [res, retInts, retFloats, retStrings, retBuffer] = sim.simxCallScriptFunction(clientID, ...
                    'ResizableFloor_5_25', ...
                    sim.sim_scripttype_childscript, ...
                    'createcube_function', ...
                    [0,1,1], ... %   [color_flag(0=NULL,1=green),
                    [1, 1, 0.3, 0.05, 0.05, 0.05], ...  %   [posx, posy, posz, sizex, sizey, sizez]
                    '', ...
                    [], ...
                    sim.simx_opmode_blocking);
            %}

            % % % % % %             Execute tasks     % % % % % %
            
            % % % Read the coordinates(position) of each device.
            % % %   [number returnCode,array position]=simxGetObjectPosition(number clientID,number objectHandle,number relativeToObjectHandle,number operationMode)
%             [res_rob_handle, rob_handle] = sim.simxGetObjectHandle(clientID,'base_link_respondable',sim.simx_opmode_blocking)
            [res_rob_pos, rob_pos] = sim.simxGetObjectPosition(clientID, rob_handle, -1, sim.simx_opmode_streaming)
            %         disp(rob_pos);
            
%             [res_tab_handle, tab_handle] = sim.simxGetObjectHandle(clientID,'customizableTable',sim.simx_opmode_blocking)
            [res_tab_pos, tab_pos] = sim.simxGetObjectPosition(clientID, tab_handle, -1, sim.simx_opmode_streaming)
            
%             [res_con_handle, con_handle] = sim.simxGetObjectHandle(clientID,'customizableConveyor',sim.simx_opmode_blocking)
            [res_con_pos, con_pos] = sim.simxGetObjectPosition(clientID, con_handle, -1, sim.simx_opmode_streaming)
            
            
            
            % let's define now the target positions needed
            fposition1 = [0.08,    0.6,    0.6,    0,  0,  0];    % [x, y, z, alpha, beta, gamma] first position
            fposition2 = [0.2,    0,      0.9,    0,  0,  0];
            fposition3 = [0.34999,0.1587, 0.63,    0,   0,    0];    % above pickup position
            fposition4 = [0.34999,0.1587, 0.561,    0,  0,  0];    % pickup position
            fposition5 = [-0.2,   0.27,   0.63,    0,  0,  0];    % above place position
            fposition6 = [tab_pos,    0,  0,  0];    % placeposition
            
            disp(fposition6)
                        pause(5);

            % % % %             Read proximity sensor
            [res_sensor_handle, Proximity_sensor_handle] = sim.simxGetObjectHandle(clientID, 'Proximity_sensor', sim.simx_opmode_blocking);
            [res_read_sensor, detectionState, detectedPoint, detectedObjectHandle] = sim.simxReadProximitySensor(clientID, Proximity_sensor_handle, sim.simx_opmode_streaming);
            
            disp(Proximity_sensor_handle);

            disp('sensor state');

            disp(res_read_sensor);
            disp(detectionState);
            disp(detectedPoint);
            disp(detectedObjectHandle);
            disp(cube0_handle);
            
%             disp(dddddd);


            if(detectionState > 0)
                [res_cube0_pos, cube0_pos] = sim.simxGetObjectPosition(clientID, cube0_handle, -1, sim.simx_opmode_streaming)
                
                fposition4 = [cube0_pos,	0,	0,	0];    % place position
                fposition3 = [cube0_pos(1), cube0_pos(2), cube0_pos(3)+0.1, 0,  0,	0];    % above place position

                moveL (clientID, motoman_target, fposition3, 2);

                


            end

            

            pause(5);
%{
            % % %             Remove a cube
            [res_cube0_remove] = sim.simxRemoveObject(clientID, cube0_handle, sim.simx_opmode_blocking)
            [res_cube1_remove] = sim.simxRemoveObject(clientID, cube1_handle, sim.simx_opmode_blocking)


            % % %             Remove model (Matlab function)  % % % % %
            % % %             Remove a robot
            [res_rob_remove] = sim.simxRemoveModel(clientID, rob_handle, sim.simx_opmode_blocking);
            % % %             Remove a conveyor
            [res_con_remove] = sim.simxRemoveModel(clientID, con_handle, sim.simx_opmode_blocking);
            % % %             Remove a table
            [res_con_remove] = sim.simxRemoveModel(clientID, tab_handle, sim.simx_opmode_blocking);
%}

        end


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
