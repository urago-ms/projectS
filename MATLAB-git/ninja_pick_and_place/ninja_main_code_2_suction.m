clear all
close all
clc
origine_table=[-0.1,0.27,0.53];
cube_dimensions=[0.04,0.04,0.04];
width_offset=0.01;
length_offset=0.03;
end_test=0;
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
simtime=zeros(100,2);
simtime2=zeros(100,2);

number=0;
cmdTime=0;
id_num=0;


% From spawn_test
disp('Program started');
% sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
% From spawn_test end


if(clientID>-1)
    disp ('connected to remote API server');
    % object handles
%     [res,j1]=vrep.simxGetObjectHandle(clientID,'RG2_active1',vrep.simx_opmode_blocking);
%     [res,j2]=vrep.simxGetObjectHandle(clientID,'RG2_active2',vrep.simx_opmode_blocking);
%     [res,motoman_target]=vrep.simxGetObjectHandle(clientID,'target',vrep.simx_opmode_blocking);
%     [res,Proximity_sensor]=vrep.simxGetObjectHandle(clientID,'Proximity_sensor',vrep.simx_opmode_blocking);
     
%     simFloat simGetSimulationTime()

        
    [res, j1]=vrep.simxGetObjectHandle(clientID,'ROBOTIQ_85_active1',vrep.simx_opmode_blocking);
    [res, j2]=vrep.simxGetObjectHandle(clientID,'ROBOTIQ_85_active2',vrep.simx_opmode_blocking);
    [res, motoman_target]=vrep.simxGetObjectHandle(clientID,'target',vrep.simx_opmode_blocking);
    [res, Proximity_sensor]=vrep.simxGetObjectHandle(clientID,'Proximity_sensor',vrep.simx_opmode_blocking);
    
%     j1 = 90;
%     j2 = 40;
    
    
    % let's define now the target positions needed
    fposition1=[0.08,    0.6,    0.6,    0,  0,  0];    % [x, y, z, alpha, beta, gamma] first position
    fposition2=[0.2,    0,      0.9,    0,  0,  0];
    fposition3=[0.34999,0.1587, 0.63,    0, 0,  0];    % above pickup position
    fposition4=[0.34999,0.1587, 0.561,    0,  0,  0];    % pickup position
    fposition5=[-0.2,   0.27,   0.63,    0,  0,  0];    % above place position
    fposition6=[-0.2,   0.27,   0.561,    0,  0,  0];    % placeposition
    
%     fposition1=[0.2,    0.6,    0.6,    0,  0,  0];    % [x, y, z, alpha, beta, gamma]
%     fposition2=[0.1,    0,      0.9,    0,  0,  0];
%     fposition3=[-0.12,  -0.3,   0.75,   0,  0,  0];    % above pickup position
%     fposition4=[-0.12,  -0.3,   0.65,   0,  0,  0];    % pickup position
%     fposition5=[0,      0.2,    0.75,   0,  0,  0];    % above place position
%     fposition6=[0,      0.2,    0.65,   0,  0,  0];    % placeposition

    
    gripper (clientID,0,j1,j2);pause(1.5);  % open gripper
    moveL (clientID, motoman_target, fposition3, 2);
    
    id_num=1;
    while(end_test==0)
        
%             	[number, cmdTime]=vrep.simxGetLastCmdTime(number, clientID)
        simtime(id_num, 1) = id_num;
        simtime(id_num, 2) = vrep.simxGetLastCmdTime(clientID);
        
%         simtime2(id_num, 1) = id_num;
%         simtime2(id_num, 2) = vrep.simxGetLastCmdTime(clientID)
        
%         [pingTime]=vrep.simxGetPingTime(clientID)
        
        [res,PSsensor_distance, detectedPoint]=vrep.simxReadProximitySensor(clientID, Proximity_sensor, vrep.simx_opmode_blocking);
      
        if(PSsensor_distance > 0)
            moveL(clientID, motoman_target, fposition4,2);
            gripper (clientID,1,j1,j2);pause(2);  % close gripper and pickup the cube
            vrep.simxSetIntegerSignal(clientID,'succtionActive',1, vrep.simx_opmode_oneshot);
            
            
             [res retInts retFloats retStrings retBuffer]=vrep.simxCallScriptFunction(clientID, ...
                                                                                'suctionPadLoopClosureDummy1', ...
                                                                                sim.sim_scripttype_childscript, ...
                                                                                'suction_function', ...
                                                                                [0,0,1],[0.1,0.3,0.68], ...
                                                                                'Hello world!', ...
                                                                                [], ...
                                                                                vrep.simx_opmode_blocking);
            
            moveL(clientID, motoman_target, fposition3,2);
            moveL(clientID, motoman_target, fposition5,2);
            moveL(clientID, motoman_target, fposition6,2);
            %             vrep.simxClearIntegerSignal(clientID,'succtionActive',vrep.simx_opmode_oneshot);
            vrep.simxSetIntegerSignal(clientID,'succtionActive',0, vrep.simx_opmode_oneshot);
            
%             vrep.simxSetLinkDummy(l,-1);
            
            gripper (clientID,0,j1,j2);pause(1);
            moveL(clientID, motoman_target, fposition5,2);
            moveL(clientID, motoman_target, fposition3,2);
            % refresh the place position
            [end_test ,fposition6, fposition5, fposition3] = pick_and_place(origine_table, 3, 3, 3, cube_dimensions, width_offset, length_offset, fposition6, fposition5, fposition3);
        end
        id_num = id_num + 1;
    end
vrep.delete();  % call the destructor
disp('program ended');
    
end

    
    
    
    
    
    