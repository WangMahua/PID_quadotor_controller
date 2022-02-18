% pos/vel/yaw controller
% 3 step: 
%   altitude control -> get thrust 
%   lateralposition control ->  get desire moment 
%   yaw control -> control yaw (in this code we don't implement this part)

% input data u :
% pos_desired 
%     3
% vel_desired 
%     3
% yaw desired 
%     1 
% pos vel  R  Omega
%  3   3   9    3 


function out = pid_controller(u,P)

persistent last_vel
if isempty(last_vel)
    last_vel = [0,0,0];
end
if P.clearpersist
    last_vel = [0,0,0];
end
% time setting
delta_t = P.Ts; 

% set input 
pos_desired = u(1:3);
vel_desired = u(4:6);
yaw_desired = u(7);
pos_now = u(8:10);
vel_now = u(11:13);
R_now = reshape(u(14:22),3,3);
omega_now = u(23:25);
t = u(26);

% init ouptut
thrust_cmd = 0;
moment_cmd = [0 0 0];
acc = [0 0 0];
acc(1)=vel_now(1) - last_vel(1); 
acc(2)=vel_now(2) - last_vel(2); 
acc(3)=vel_now(3) - last_vel(3); 
last_vel = vel_now;
acc = acc/delta_t;

uav_state = [u(8) u(9) u(10) u(11) u(12) u(13) acc(1) acc(2) acc(3) 1];

% calculate thrust
thrust_cmd = altitude_control(pos_desired(3),vel_desired(3),acc(3),pos_now(3),vel_now(3),R_now,delta_t,P);

% calculate 
acc_desired = LateralPositionControl(pos_desired,vel_desired,pos_now,vel_now,acc,P);

%ecbf 
acc_ecbf = acc_desired;
acc_ecbf = ecbf(acc_ecbf,thrust_cmd,yaw_desired,uav_state,u(27:36),u(37:46),u(47:56),u(57:66),u(67:76),P);

% calculate moment
% omega_desired = RollPitchControl(acc_desired,R_now,thrust_cmd,P);
omega_desired = RollPitchControl(acc_ecbf,R_now,thrust_cmd,P);
moment_cmd = BodyRateControl(omega_desired,omega_now,P);


% moment_cmd = [0,0,0];
last_v = [vel_now(1) vel_now(2) vel_now(3)] ;
acc_print = [acc_desired(1) acc_desired(2) acc_desired(3)];
acc_ecbf_print = [acc_ecbf(1) acc_ecbf(2) acc_ecbf(3)];
out = [thrust_cmd moment_cmd acc_print acc_ecbf_print last_v];

P.clearpersist = false;

end

