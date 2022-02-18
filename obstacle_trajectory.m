function out = obstacle_trajectory(u,P)
t = u(end);

% state calculate 
% 
persistent obstacle1_state
persistent obstacle2_state
persistent obstacle3_state
persistent obstacle4_state

if isempty(obstacle1_state)
    obstacle1_state = [3 5 -1 0 0 0 0 0 0 1.5];
    obstacle2_state = [14 12 -1 0 0 0 0 0 0 1.5];
    obstacle3_state = [10 2 -1 0 0 0 0 0 0 0.5];
    obstacle4_state = [4 12 -1 0 0 0 0 0 0 0.5];
end
if t < P.Ts
    obstacle1_state = [3 5 -1 0 0 0 0 0 0 1.5];
    obstacle2_state = [14 12 -1 0 0 0 0 0 0 1.5];
    obstacle3_state = [10 2 -1 0 0 0 0 0 0 0.5];
    obstacle4_state = [4 12 -1 0 0 0 0 0 0 0.5];
end
% change vel
% if t <5 
%     obstacle1_state(4:6) = [0.5 -0.5 0];
%     obstacle2_state(4:6) = [-0.5 0.5 0];
% elseif t>5 && t<10 
%     obstacle1_state(4:6) = [-0.5 0.5 0];
%     obstacle2_state(4:6) = [0.5 -0.5 0]; 
% elseif t>10 && t<15 
%     obstacle1_state(4:6) = [0.5 -0.5 0];
%     obstacle2_state(4:6) = [-1 1 0];   
% elseif t>15 && t<20 
%     obstacle1_state(4:6) = [-1 1 0];
%     obstacle2_state(4:6) = [1 -1 0]; 
% end
obstacle1_state(4:6) = [sin(t) 0 0];
obstacle2_state(4:6) = [cos(t) 0 0]; 
    
obstacle1_state(1:3) = obstacle1_state(1:3) + obstacle1_state(4:6)*P.Ts;
obstacle2_state(1:3) = obstacle2_state(1:3) + obstacle2_state(4:6)*P.Ts;
P.obstacleclearpersist = false;
out = [obstacle1_state obstacle2_state obstacle3_state obstacle4_state];

end