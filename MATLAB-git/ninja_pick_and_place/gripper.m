function gripper (clientID, closing, j1, j2)
vrep=remApi('remoteApi');
[r,p1]=vrep.simxGetJointPosition(clientID, j1, vrep.simx_opmode_blocking);
r
[r,p2]=vrep.simxGetJointPosition(clientID, j2, vrep.simx_opmode_blocking);

p1
p2
r
% if (closing==1)
%     if (p1<(p2-0.108))
%         vrep.simxSetJointTargetVelocity (clientID, j1, -0.01, vrep.simx_opmode_blocking);
%         vrep.simxSetJointTargetVelocity (clientID, j2, -0.08, vrep.simx_opmode_blocking);
%         disp ('11111111111111');
%     else
%         vrep.simxSetJointTargetVelocity (clientID, j1, -0.08, vrep.simx_opmode_blocking);
%         vrep.simxSetJointTargetVelocity (clientID, j2, -0.08, vrep.simx_opmode_blocking);
%         disp ('2222222222222');
%     end
% else
%     if(p1<p2)
%         vrep.simxSetJointTargetVelocity (clientID, j1, 0.08, vrep.simx_opmode_blocking);
%         vrep.simxSetJointTargetVelocity (clientID, j2, 0.02, vrep.simx_opmode_blocking);
%         disp ('3333333333333333333');
%     else
%         vrep.simxSetJointTargetVelocity (clientID, j1, 0.02, vrep.simx_opmode_blocking);
%         vrep.simxSetJointTargetVelocity (clientID, j2, 0.08, vrep.simx_opmode_blocking);
%         disp ('44444444444');
%     end
% end
% 
% end



% if (closing==1)
%     if (p1<(p2-0.108))
%         vrep.simxSetJointTargetVelocity (clientID, j1, -0.01, vrep.simx_opmode_blocking);
%         vrep.simxSetJointTargetVelocity (clientID, j2, -0.04, vrep.simx_opmode_blocking);
%     else
%         vrep.simxSetJointTargetVelocity (clientID, j1, -0.04, vrep.simx_opmode_blocking);
%         vrep.simxSetJointTargetVelocity (clientID, j2, -0.04, vrep.simx_opmode_blocking);
%     end
% else
%     if(p1<p2)
%         vrep.simxSetJointTargetVelocity (clientID, j1, 0.04, vrep.simx_opmode_blocking);
%         vrep.simxSetJointTargetVelocity (clientID, j2, 0.02, vrep.simx_opmode_blocking);
%     else
%         vrep.simxSetJointTargetVelocity (clientID, j1, 0.02, vrep.simx_opmode_blocking);
%         vrep.simxSetJointTargetVelocity (clientID, j2, 0.04, vrep.simx_opmode_blocking);
%     end
% end
% 
% end



if (closing==1)
    if (p1<(p2-0.008))
        vrep.simxSetJointTargetVelocity (clientID, j1, -0.04, vrep.simx_opmode_blocking);
        vrep.simxSetJointTargetVelocity (clientID, j2, -0.04, vrep.simx_opmode_blocking);
         disp ('11111111111111');
    else
        vrep.simxSetJointTargetVelocity (clientID, j1, -0.01, vrep.simx_opmode_blocking);
        vrep.simxSetJointTargetVelocity (clientID, j2, -0.04, vrep.simx_opmode_blocking);
         disp ('2222222222222');
    end
else
    if(p1<p2)
        vrep.simxSetJointTargetVelocity (clientID, j1, 0.04, vrep.simx_opmode_blocking);
        vrep.simxSetJointTargetVelocity (clientID, j2, 0.02, vrep.simx_opmode_blocking);
         disp ('33333333333333');
    else
        vrep.simxSetJointTargetVelocity (clientID, j1, 0.02, vrep.simx_opmode_blocking);
        vrep.simxSetJointTargetVelocity (clientID, j2, 0.04, vrep.simx_opmode_blocking);
         disp ('444444444444');
    end
end

end


% % ORIGIN
% if (closing==1)
%     if (p1<(p2-0.008))
%         vrep.simxSetJointTargetVelocity (clientID, j1, -0.01, vrep.simx_opmode_blocking);
%         vrep.simxSetJointTargetVelocity (clientID, j2, -0.04, vrep.simx_opmode_blocking);
%          disp ('11111111111111');
%     else
%         vrep.simxSetJointTargetVelocity (clientID, j1, -0.04, vrep.simx_opmode_blocking);
%         vrep.simxSetJointTargetVelocity (clientID, j2, -0.04, vrep.simx_opmode_blocking);
%          disp ('2222222222222');
%     end
% else
%     if(p1<p2)
%         vrep.simxSetJointTargetVelocity (clientID, j1, 0.04, vrep.simx_opmode_blocking);
%         vrep.simxSetJointTargetVelocity (clientID, j2, 0.02, vrep.simx_opmode_blocking);
%          disp ('33333333333333');
%     else
%         vrep.simxSetJointTargetVelocity (clientID, j1, 0.02, vrep.simx_opmode_blocking);
%         vrep.simxSetJointTargetVelocity (clientID, j2, 0.04, vrep.simx_opmode_blocking);
%          disp ('444444444444');
%     end
% end
% 
% end