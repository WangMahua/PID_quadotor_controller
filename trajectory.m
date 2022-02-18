function out = pid_trajectory(u,P)

% input(1*1) : time
% output(6*1) : desired trajectory and desired yaw angle
% state(10*1) : pos vel acc R



% state calculate 
persistent target_last_pos 
persistent target_last_vel
persistent target_state

if isempty(target_last_pos)
    target_last_pos = [0 0 0];
    target_last_vel = [0 0 0];
    target_state = [0 0 -1 1 1 0 0 0 0 1];
end

if P.clearpersist
    target_last_pos = [0 0 0];
    target_last_vel = [0 0 0];
    target_state = [0 0 -1 1 1 0 0 0 0 1];
end

% time
dt = 0.01;
t = u(end);

% update state 
% supose constant vel [1 1 0]

vel = target_state(4:6);
target_state(1:3) = (t)*vel;

% update command 
if t <20 
    p_d = [target_state(1)  target_state(2)  -1];
else
    p_d = [(20)*vel(1)  (20)*vel(2)  -1];
end 

% p_d = [1 1  -1];
v_d = [0.0  0.0  0.0]; 
yaw_d = 0.0;


out = [p_d v_d yaw_d target_state];


end
