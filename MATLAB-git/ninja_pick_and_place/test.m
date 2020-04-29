clear all
close all
clc
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    
    if (clientID>-1)
        disp('Connected to remote API server');
        % create some joints pos
        joint_pos1=[-2*pi/3,0,0,0,pi/4,0];
        joint_pos2=[0,-4/pi,pi/3,0,0,0];
        joint_pos3=[pi,pi/2,5*pi/2,0,0,0];
        joint_pos4=[0,0,0,0,0,0];

        %joints handles
        h=[0,0,0,0,0,0]
        [r,h(1)]=vrep.simxGetObjectHandle(clientID,'rotary_head',vrep.simx_opmode_blocking);
        [r,h(2)]=vrep.simxGetObjectHandle(clientID,'lower_arm',vrep.simx_opmode_blocking);
        [r,h(3)]=vrep.simxGetObjectHandle(clientID,'upper_arm',vrep.simx_opmode_blocking);
        [r,h(4)]=vrep.simxGetObjectHandle(clientID,'forearm_twisting',vrep.simx_opmode_blocking);
        [r,h(5)]=vrep.simxGetObjectHandle(clientID,'wrist',vrep.simx_opmode_blocking);
        [r,h(6)]=vrep.simxGetObjectHandle(clientID,'axis6',vrep.simx_opmode_blocking);

        while true
            for i=1:6
            vrep.simxSetJointTargetPosition(clientID,h(i),joint_pos1(i),vrep.simx_opmode_streaming)
            end
            pause(5);
            
            for i=1:6
            vrep.simxSetJointTargetPosition(clientID,h(i),joint_pos2(i),vrep.simx_opmode_streaming)
            end
            pause(5);
            
            for i=1:6
            vrep.simxSetJointTargetPosition(clientID,h(i),joint_pos3(i),vrep.simx_opmode_streaming)
            end
            pause(5);
            
            for i=1:6
            vrep.simxSetJointTargetPosition(clientID,h(i),joint_pos4(i),vrep.simx_opmode_streaming)
            end
            pause(5);
        end
        
    else
        disp('Failed connecting to remote API server');
    end
        vrep.delete(); % call the destructor!
    
        disp('Program ended');