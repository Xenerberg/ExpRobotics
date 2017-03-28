close all;clc;
disp('Program started');
%vrep = remApi('remoteApi','extApi.h'); %Using header
vrep = remApi('remoteApi');
vrep.simxFinish(-1); %Close connections that exist.
clientID = vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
q_target_pos = zeros(6,1);
if clientID > -1
   disp('Connected to V-REP remote API server');
   %Print to V-Rep that Client is connected
   vrep.simxAddStatusbarMessage(clientID,'Server connected to MATLAB',vrep.simx_opmode_blocking);
   [~,Objects_Scene, ~, ~, ObjectsInScene] = vrep.simxGetObjectGroupData(clientID,vrep.sim_appobj_object_type,0,vrep.simx_opmode_blocking); 
   ObjectsInScene = ObjectsInScene(:,1);
   h_joints = zeros(6,1);
   ind_joints = [14:3:27,28];
   ind_joints = [18:3:30,32];
   h_joints = Objects_Scene(ind_joints);
   h_target = Objects_Scene(45);
   h_ee = Objects_Scene(43);
   %Enable the syncrhonous mode
   vrep.simxSynchronous(clientID,true)
   
   %Start simulation 
   vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
  
   
   pause(1);
   counterStep = 0;
   totalTime = 200;
   %Fetch the end-effector pose
   [rtrn,ee_pos] = vrep.simxGetObjectPosition(clientID,h_ee,-1,vrep.simx_opmode_blocking);
   [rtrn,ee_anglesVREP] = vrep.simxGetObjectOrientation(clientID,h_ee,-1,vrep.simx_opmode_blocking);
   R = angle2dcm(ee_anglesVREP(1),ee_anglesVREP(2),ee_anglesVREP(3),'XYZ')';       
   g_curr = [R,ee_pos';zeros(1,3),1];

   %Fetch target pose
   [rtrn,ee_targ] = vrep.simxGetObjectPosition(clientID,h_target,-1,vrep.simx_opmode_blocking);
   [rtrn,ee_anglesTarg] = vrep.simxGetObjectOrientation(clientID,h_target,-1,vrep.simx_opmode_blocking);
   R = angle2dcm(ee_anglesTarg(1),ee_anglesTarg(2),ee_anglesTarg(3),'XYZ')';       
   g_targ = [R,ee_targ';zeros(1,3),1];
   theta = zeros(6,1);
   theta_des = [pi/3;pi/4;-pi/6;pi/4;pi/3;0]; %theta_des = [0;pi/4;0;0;0;0];
   lambda = 1e-6;
   Kp = 25;
   Kv = 10;
   counter = 0;
   max_dq = 1000*ones(6,1);
   for iCount = 1:6
      [~] = vrep.simxSetJointTargetVelocity(clientID,h_joints(iCount),0,vrep.simx_opmode_blocking); 
   end
   while(1)
       counter = counter + 1;
       %Run a time-step
       vrep.simxSynchronousTrigger(clientID); 
       
       %Fetch Joint-space force
       for iCount = 1:6
           [~,tau_t(iCount,1)] = vrep.simxGetJointForce(clientID,h_joints(iCount),vrep.simx_opmode_blocking);
           [~,q_curr(iCount,1)] = vrep.simxGetJointPosition(clientID,h_joints(iCount),vrep.simx_opmode_blocking);
           [~,dq(iCount,1)] = vrep.simxGetObjectFloatParameter(clientID, h_joints(iCount),2012,vrep.simx_opmode_blocking);           
       end              
       g_curr = fn_CreateForwardKinExp(q_curr,om,q)*g_st_0;
       %tau_comp(counter,:) = fn_ComputeJSg(theta,om,q);       
       tau(counter,:) = tau_t;
       
       
       J = fn_Jacobian(q_curr,om,q,0,g_st_0,0);
       J_star =  J'*inv(J*J' + lambda*eye(6,6));
       del_g = g_targ*inv(g_curr);       
       ang = [-del_g(2,3);del_g(1,3);-del_g(1,2)];
       lin = del_g(1:3,4);
       screw = [lin;ang];
       theta_dot_des = J_star*screw;
       theta_des = q_curr + 0.05*theta_dot_des;
       
       
       del_theta = theta_des - q_curr;
       del_theta_dot = theta_dot_des - dq;       
       u = Kp*del_theta + Kv*del_theta_dot; %
       
       %theta_dot = theta_dot + 0.05*u;
       
       
       M = fn_CreateMassMatrix(q_curr,om,q);
       g_q  = fn_ComputeJSg(q_curr,om,q);       
       
       tau_cmd = M*u + g_q;tau_cmd = tau_cmd.*-1 ;
       %tau_cmd = g_q;
       for iCount = 1:6
           if sign(tau_cmd(iCount))*sign(tau(counter,iCount)) < 0
               max_dq(iCount) = max_dq(iCount)*-1;
           end
           [~] = vrep.simxSetJointTargetVelocity(clientID,h_joints(iCount),max_dq(iCount),vrep.simx_opmode_blocking);
           [~] = vrep.simxSetJointForce(clientID, h_joints(iCount), abs(tau_cmd(iCount)), vrep.simx_opmode_blocking);
       end
       
       
       
       if counter > 400
           break;
       end
   end
   
   %Stop simulation
   vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
   
   %Close connection
   vrep.simxFinish(clientID);
end



