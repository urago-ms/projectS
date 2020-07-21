% This example illustrates how to execute complex commands from
% a remote API client. You can also use a similar construct for
% commands that are not directly supported by the remote API.
%
% Load the demo scene 'remoteApiCommandServerExample.ttt' in CoppeliaSim, then 
% start the simulation and run this program.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

function spawn_test()

    disp('Program started');
    % sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');
        
% % %         Repeated creation and deletion of the device.
        for count = 1:2

%{            
% % % % % %             Calling a function in the threaded child script of ResizableFloor_5_25 to create a facility
            % % % Genetate a manipulator
            [res retInts retFloats retStrings retBuffer]=sim.simxCallScriptFunction(clientID, ...
                'ResizableFloor_5_25', ...
                sim.sim_scripttype_childscript, ...
                'spawn_function', ...
                [],[1.5,1.4,0], ...
                'Hello world!', ...
                [], ...
                sim.simx_opmode_blocking);
            %     pause(3);
            if (res==sim.simx_return_ok)
                fprintf('Returned message: %s\n',retStrings);
            else
                fprintf('Remote function call failed\n');
            end
            
            % % % Genetate a conveyor
            [res retInts retFloats retStrings retBuffer]=sim.simxCallScriptFunction(clientID, ...
                'ResizableFloor_5_25', ...
                sim.sim_scripttype_childscript, ...
                'spawn_function_con', ...
                [0,0,1],[0.1,0.3,0.68], ...
                'Hello world!', ...
                [], ...
                sim.simx_opmode_blocking);
            %         pause(3);
            
            if (res==sim.simx_return_ok)
                fprintf('Returned message: %s\n',retStrings);
            else
                fprintf('RRRRRRRRRemote function call failed\n');
            end
            
            % % % Genetate a table
            [res retInts retFloats retStrings retBuffer]=sim.simxCallScriptFunction(clientID, ...
                'ResizableFloor_5_25', ...
                sim.sim_scripttype_childscript, ...
                'spawn_function_tab', ...
                [],[-1.0,1.0,0.68], ...
                'Hello world!', ...
                [], ...
                sim.simx_opmode_blocking);
            pause(1);
            
            
            if (res==sim.simx_return_ok)
                fprintf('Returned message: %s\n',retStrings);
            else
                fprintf('tttttttttttRRRemote function call failed\n');
            end
            
            % 4. First send a command to display a specific message in a dialog box:
            %         [res retInts retFloats retStrings retBuffer]=sim.simxCallScriptFunction(clientID, ...
            %                                                                                 'ResizableFloor_5_25', ...
            %                                                                                 sim.sim_scripttype_childscript, ...
            %                                                                                 'remove_function_rob', ...
            %                                                                                 [],[-1.0,1.0,0.68], ...
            %                                                                                 'Hello world!', ...
            %                                                                                 [], ...
            %                                                                                 sim.simx_opmode_blocking);
%}
            
% % % % %             Generate facilities using MATLAB functions
% % %             Genarate a robot
            [res_rob_genetate, rob_handle] = sim.simxLoadModel(clientID,'motoman_HP3J_with_base.ttm', 0, sim.simx_opmode_blocking)
%           [res_rob_genetate, rob_handle]=simxLoadModel(clientID, "motoman_HP3J.ttm", "", sim.simx_opmode_blocking)
            [res_rob_setpos] = sim.simxSetObjectPosition(clientID, rob_handle, -1, [-0.6 0.65 0.15], sim.simx_opmode_oneshot)
            
% % %             Genarate a conveyor
            [res_con_genetate, con_handle] = sim.simxLoadModel(clientID,'customizable conveyor belt.ttm', 0, sim.simx_opmode_blocking)
            [res_con_setpos] = sim.simxSetObjectPosition(clientID, con_handle, -1, [0 0.5 0.45], sim.simx_opmode_oneshot)

% % %             Genarate a table
            [res_tab_genetate, tab_handle] = sim.simxLoadModel(clientID,'customizable table.ttm', 0, sim.simx_opmode_blocking)
            [res_tab_setpos] = sim.simxSetObjectPosition(clientID, tab_handle, -1, [-0.6 0.65 0.45], sim.simx_opmode_oneshot)
            
            pause(5);

% % %             Remove model (Matlab function)
% % %             Remove a robot
            [res_rob_remove] = sim.simxRemoveModel(clientID, rob_handle, sim.simx_opmode_blocking);
            
% % %             Remove a conveyor
            [res_con_remove] = sim.simxRemoveModel(clientID, con_handle, sim.simx_opmode_blocking);

% % %             Remove a table
            [res_con_remove] = sim.simxRemoveModel(clientID, tab_handle, sim.simx_opmode_blocking);

            
            
            
            
            
            
%{
%             pause(1);
% % % % % % Remove model (coppeliasim threaded child script function)
            % % % % % % Remove manipulator model
            fprintf('remove manipulator\n');
%             [res_rob_remove, rob_handle] = sim.simxGetObjectHandle(clientID,'motoman_HP3J_base_link_respondable',sim.simx_opmode_blocking);
            [rob_remove_flag] = sim.simxRemoveModel(clientID, rob_handle, sim.simx_opmode_blocking);
            
%             disp(res_rob_remove);
            disp(rob_handle);
            
            fprintf('rob_remove_flag = \n');
            disp(rob_remove_flag);
            
%             pause(1);
            % % % % %         Remove table model
            fprintf('remove customizableTable\n');
            [res_tab_remove, tab_handle] = sim.simxGetObjectHandle(clientID,'customizableTable',sim.simx_opmode_blocking);
            [tab_remove_flag] = sim.simxRemoveModel(clientID, tab_handle, sim.simx_opmode_blocking);
            
            disp(res_tab_remove);
            disp(tab_handle);
            
            fprintf('tab_remove_flag = \n');
            disp(tab_remove_flag);
            
            
%             pause(1);
            % % % %          Remove conveyor model
            fprintf('remove customizableConveyor\n');
            [res_con_remove, con_handle] = sim.simxGetObjectHandle(clientID,'customizableConveyor',sim.simx_opmode_blocking);
            [con_remove_flag] = sim.simxRemoveModel(clientID, con_handle, sim.simx_opmode_blocking);
            
            disp(res_con_remove);
            disp(con_handle);
            
            fprintf('con_remove_flag = \n');
            disp(con_remove_flag);
%}
        end


        
        
        

                                                                            
%         
%         [res1, tab]=sim.simxGetObjectHandle(clientID,'customizable table.ttm',sim.simx_opmode_blocking);
%                 pause(1);
% %         codeA = sim.simxRemoveObject(clientID,rob,sim.simx_opmode_blocking);
%         fprintf('ddddd\n');
%         
%         [returnCode, prop] = sim.simxGetModelProperty(clientID, rob, sim.simx_opmode_blocking)
%         
%         
% %         codeAB = sim.simxSetModelProperty(clientID, tab, 61441, sim.simx_opmode_oneshot);
%         %         simxmodelproperty_not_visible
% %         [returnCode, prop] = sim.simxGetModelProperty(clientID, tab, sim.simx_opmode_blocking)
% 
%         [returnCode, position] = sim.simxGetObjectPosition(clientID,tab,-1,sim.simx_opmode_blocking)
%         
%         codeAC = sim.simxSetObjectPosition(clientID,tab,-1,[-1,2,1],sim.simx_opmode_blocking)
% 
%         [res1, con]=sim.simxGetObjectHandle(clientID,'customizable conveyor belt.ttm',sim.simx_opmode_blocking);
%         codeB = sim.simxRemoveModel(clientID,con,sim.simx_opmode_blocking);
%         
%         [res1, tab]=sim.simxGetObjectHandle(clientID,'customizable table.ttm',sim.simx_opmode_blocking);
%         codeC = sim.simxRemoveModel(clientID,tab,sim.simx_opmode_blocking);
        
        if (res == sim.simx_return_ok)
            fprintf('Returned message: %s\n',retStrings);
        else
            fprintf('3333333Remote function call failed\n');
        end
%{
        % 2. Now create a dummy object at coordinate 0.1,0.2,0.3 with name 'MyDummyName':
        [res retInts retFloats retStrings retBuffer]=sim.simxCallScriptFunction(clientID,'spawn_test',sim.sim_scripttype_childscript,'createDummy_function',[],[0.1 0.2 0.3],'MyDummyName',[],sim.simx_opmode_blocking);
        if (res==sim.simx_return_ok)
            fprintf('Dummy handle: %d\n',retInts(1));
        else
            fprintf('Remote function call failed\n');
        end

        % 3. Now send a code string to execute some random functions:
        code=['local octreeHandle=simCreateOctree(0.5,0,1)', char(10), ...
            'simInsertVoxelsIntoOctree(octreeHandle,0,{0.1,0.1,0.1},{255,0,255})', char(10), ...
            'return ''done'''];
        [res retInts retFloats retStrings retBuffer]=sim.simxCallScriptFunction(clientID,'spawn_test',sim.sim_scripttype_childscript,'executeCode_function',[],[],code,[],sim.simx_opmode_blocking);
        if (res==sim.simx_return_ok)
            fprintf('Code execution returned: %s\n',retStrings);
        else
            fprintf('Remote function call failed\n');
        end
%}

        % Now close the connection to CoppeliaSim:    
        sim.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
    sim.delete(); % call the destructor!
    
    disp('Program ended');
end
