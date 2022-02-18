clc;clear;close all;

% declare all parameter of the quadrotor and initial condition

% Sample time of the controller
P.Ts = 0.01;

% Sample time of the state plotter
P.plotTs = 0.1;

% physical parameters of airframe
P.gravity = 9.81;   % [m/s/s]
P.mass    = 4.34;   % [kg]
P.Jxx     = 0.0820; % [kg-m2]
P.Jyy     = 0.0845; % [kg-m2]
P.Jzz     = 0.1377; % [kg-m2]

% The dist from CoM to the center of ea. rotor in the b1-b2 plane
P.d  = 0.315; % [m]

% actuator constant
P.c_tauf = 8.004e-3; % [m]

% Mixing matrix that relates thrust/moments to actuators
P.Mix = inv([1 1 1 1; 0 -P.d 0 P.d;...
         P.d 0 -P.d 0; -P.c_tauf P.c_tauf -P.c_tauf P.c_tauf]);

% first cut at initial conditions
P.p0 = [0 0 0];
P.v0 = [0 0 0];
P.R0 = [1 0 0; 0 1 0; 0 0 1];
% P.R0 = expm(hat(deg2rad([178.2 0 0])));
P.Omega0 = deg2rad([0 0 0]);

% sketch parameters
P.nRotors = 4;

% time constant for dirty derivative filter
P.tau = 0.05;

% Control gains (taken from Lee2011, arXiv:1003.2005v4)
P.kx = 16*P.mass;
P.kv = 5.6*P.mass;
P.kR = 8.81;
P.kOmega = 2.54;

P.kx = 4*P.mass;
P.kv = 5.6*P.mass;
P.kR = 8.81;
P.kOmega = 2.54;


P.J = [P.Jxx 0 0;...
       0 P.Jyy 0;...
       0 0 P.Jzz];
P.R_d_last = 0;
P.omega_d_last = 0;

P.acc = [0 0 0];

%PID paramwter
% for altitude control
P.kp_pos_x = 0.4;
P.kp_pos_y = 0.4;
P.kp_pos_z = 1;
%P.kd_vel_x = 0.6;
%P.kd_vel_y = 0.4;
%P.kd_vel_z = 0.4;
P.ki_pos_x = 0.001;
P.ki_pos_y = 0.001;
P.ki_pos_z = 0;

%for lateral control
P.kp_pos_xy = 0.4;
P.kp_vel_xy = 0.4;
P.max_vel_xy = 500000;
P.max_acc_xy = 500000;

P.kp_vel_z = 1;

P.integratedAltitudeError  = 0;

P.kp_pqr_x = 10;
P.kp_pqr_y = 10;
P.kp_pqr_z = 10;

P.kp_bank = 10;

% uav radius 
P.r = [1.0];
P.simutime = 25;
P.clearpersist = true;
P.obstacleclearpersist = true;
debugsim = sim('pid_ecbf_sim',P.simutime);      
%debugsim = sim('pid_ecbf_sim_debug',50);

save('simout.mat','debugsim');

%% plotting
clc;clear;close all;
load('simout.mat')

% figure(1);
% plot(debugsim.p_d.Time,debugsim.p_d.Data(:,3),'DisplayName','cmd z'); hold on;
% plot(debugsim.output_pos.Time,debugsim.output_pos.Data(:,3),'DisplayName','out z'); hold on;
% legend;
% figure(2);
% plot(debugsim.p_d.Time,debugsim.p_d.Data(:,2),'DisplayName','cmd y'); hold on;
% plot(debugsim.output_pos.Time,debugsim.output_pos.Data(:,2),'DisplayName','out y'); hold on;
% legend;
% figure(3);
% plot(debugsim.p_d.Time,debugsim.p_d.Data(:,1),'DisplayName','cmd x'); hold on;
% plot(debugsim.output_pos.Time,debugsim.output_pos.Data(:,1),'DisplayName','out x'); hold on;
% legend;
figure(1);

subplot(3,1,1);
plot(debugsim.p_d.Time,debugsim.p_d.Data(:,3),'DisplayName','target z'); hold on;
plot(debugsim.output_pos.Time,debugsim.output_pos.Data(:,3),'DisplayName','uav z'); hold on;
legend;
title('position-x');
subplot(3,1,2);
plot(debugsim.p_d.Time,debugsim.p_d.Data(:,2),'DisplayName','target y'); hold on;
plot(debugsim.output_pos.Time,debugsim.output_pos.Data(:,2),'DisplayName','uav y'); hold on;
legend;
title('position-y');
subplot(3,1,3);
plot(debugsim.p_d.Time,debugsim.p_d.Data(:,1),'DisplayName','target x'); hold on;
plot(debugsim.output_pos.Time,debugsim.output_pos.Data(:,1),'DisplayName','uav x'); hold on;
legend;
title('position-z');

% figure(4);
% plot(debugsim.PID_f,'DisplayName','f'); hold on;
% figure(5);
% plot(debugsim.PID_M.time,debugsim.PID_M.Data(:,1),'DisplayName','roll'); hold on;
% legend;
% figure(6);
% plot(debugsim.PID_M.time,debugsim.PID_M.Data(:,2),'DisplayName','pitch'); hold on;
% legend;
% figure(7);
% plot(debugsim.PID_M.time,debugsim.PID_M.Data(:,3),'DisplayName','yaw'); hold on;
% legend;
figure(8);
subplot(3,1,1);
plot(debugsim.acc.time,debugsim.acc.Data(:,1),'DisplayName','desired acc x'); hold on;
plot(debugsim.acc.time,debugsim.acc_ecbf.Data(:,1),'DisplayName','acc ecbf x'); hold on;
legend;
title('acceleration-x');
subplot(3,1,2);
plot(debugsim.acc.time,debugsim.acc.Data(:,2),'DisplayName','desired acc y'); hold on;
plot(debugsim.acc.time,debugsim.acc_ecbf.Data(:,2),'DisplayName','acc ecbf y'); hold on;
legend;
title('acceleration-y');
subplot(3,1,3);
plot(debugsim.acc.time,debugsim.acc.Data(:,3),'DisplayName','desired acc z'); hold on;
plot(debugsim.acc.time,debugsim.acc_ecbf.Data(:,3),'DisplayName','acc ecbf z'); hold on;
legend;
title('acceleration-z');
% figure(11);
% subplot(3,3,1);
% tmp = squeeze(debugsim.output_R.Data(1,1,:));
% plot(debugsim.output_R.time,tmp,'DisplayName','R11');
% subplot(3,3,2);
% tmp = squeeze(debugsim.output_R.Data(1,2,:));
% plot(debugsim.output_R.time,tmp,'DisplayName','R12');
% subplot(3,3,3);
% tmp = squeeze(debugsim.output_R.Data(1,3,:));
% plot(debugsim.output_R.time,tmp,'DisplayName','R13');
% subplot(3,3,4);
% tmp = squeeze(debugsim.output_R.Data(2,1,:));
% plot(debugsim.output_R.time,tmp,'DisplayName','R21');
% subplot(3,3,5);
% tmp = squeeze(debugsim.output_R.Data(2,2,:));
% plot(debugsim.output_R.time,tmp,'DisplayName','R22');
% subplot(3,3,6);
% tmp = squeeze(debugsim.output_R.Data(2,3,:));
% plot(debugsim.output_R.time,tmp,'DisplayName','R23');
% subplot(3,3,7);
% tmp = squeeze(debugsim.output_R.Data(3,1,:));
% plot(debugsim.output_R.time,tmp,'DisplayName','R31');
% subplot(3,3,8);
% tmp = squeeze(debugsim.output_R.Data(3,2,:));
% plot(debugsim.output_R.time,tmp,'DisplayName','R32');
% subplot(3,3,9);
% tmp = squeeze(debugsim.output_R.Data(3,3,:));
% plot(debugsim.output_R.time,tmp,'DisplayName','R33');
% plot(debugsim.v_d.Time,debugsim.v_d.Data(:,3)); hold on;
% plot(debugsim.yaw_d); hold on;
% figure(12);
% subplot(3,1,1);
% plot(debugsim.acc.time,debugsim.last_v.Data(:,1),'DisplayName','last_v x'); hold on;
% subplot(3,1,2);
% plot(debugsim.acc.time,debugsim.last_v.Data(:,2),'DisplayName','last_v y'); hold on;
% subplot(3,1,3);
% plot(debugsim.acc.time,debugsim.last_v.Data(:,3),'DisplayName','last_v z'); hold on;

%% animation 

figure(13);
s = 1500;
scatter3(debugsim.obstacle_1.Data(1,1),debugsim.obstacle_1.Data(1,2),debugsim.obstacle_1.Data(1,3),s,'green'); hold on;
scatter3(debugsim.obstacle_2.Data(1,1),debugsim.obstacle_2.Data(1,2),debugsim.obstacle_2.Data(1,3),s,'black'); hold on;
scatter3(debugsim.output_pos.Data(:,1), debugsim.output_pos.Data(:,2), debugsim.output_pos.Data(:,3),'o');hold on ;
scatter3(debugsim.p_d.Data(:,1), debugsim.p_d.Data(:,2), debugsim.p_d.Data(:,3),'*');hold on ;
title('overall trajectory');
%animate 

tmp = debugsim.acc.time;
s = 450;
s_1 = 10;
axis([0, 25, 0, 25 ,-4,0]);
for i=1:int32(size(tmp,1)/10)-1
    idx = i*10;
    figure(14);
    
    subplot(1,2,1);
    scatter3(debugsim.obstacle_1.Data(idx,1),debugsim.obstacle_1.Data(idx,2),debugsim.obstacle_1.Data(idx,3),s,'green'); hold on;
    scatter3(debugsim.obstacle_2.Data(idx,1),debugsim.obstacle_2.Data(idx,2),debugsim.obstacle_2.Data(idx,3),s,'black'); hold on;
    scatter3(debugsim.output_pos.Data(1:idx,1), debugsim.output_pos.Data(1:idx,2), debugsim.output_pos.Data(1:idx,3),s_1,"red"); hold on;
    scatter3(debugsim.p_d.Data(1:idx,1), debugsim.p_d.Data(1:idx,2), debugsim.p_d.Data(1:idx,3),s_1,"blue"); hold on;
    axis([0, 25, 0, 25 ,-4,0]);
    axis equal;
    hold off;
    
    subplot(1,2,2);
    scatter3(debugsim.obstacle_1.Data(idx,1),debugsim.obstacle_1.Data(idx,2),debugsim.obstacle_1.Data(idx,3),s,'green'); hold on;
    scatter3(debugsim.obstacle_2.Data(idx,1),debugsim.obstacle_2.Data(idx,2),debugsim.obstacle_2.Data(idx,3),s,'black'); hold on;
    scatter3(debugsim.output_pos.Data(1:idx,1), debugsim.output_pos.Data(1:idx,2), debugsim.output_pos.Data(1:idx,3),s_1,"red"); hold on;
    scatter3(debugsim.p_d.Data(1:idx,1), debugsim.p_d.Data(1:idx,2), debugsim.p_d.Data(1:idx,3),s_1,"blue"); hold on;
    view(0,90);
    axis([0, 25, 0, 25 ,-4,0]);
    
    pause(0.00001);
    hold off;
end
