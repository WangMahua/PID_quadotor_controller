function out = ecbf(acc_d,f_d,yaw,uav_state,t_st,o1_st,o2_st,o3_st,o4_st,P)
acc_d(3) = f_d/P.mass - P.gravity;
% state define 
% obstacle 
Po1 = o1_st(1:3);
Vo1 = o1_st(4:6); 
Ao1 = o1_st(7:9); 
% Vo1 = (Po1-last_Po1)/P.Ts; 
% Ao1 = (Vo1-last_Vo1)/P.Ts;
% last_Po1 = Po1;
% last_Vo1 = Vo1;
Ro1 = o1_st(end);

Po2 = o2_st(1:3);
Vo2 = o2_st(4:6); 
Ao2 = o2_st(7:9); 
% Vo2 = (Po2-last_Po2)/P.Ts; 
% Ao2 = (Vo2-last_Vo2)/P.Ts;
% last_Po2 = Po2;
% last_Vo2 = Vo2;
Ro2 = o2_st(end);

Po3 = o3_st(1:3);
Vo3 = o3_st(4:6); 
Ao3 = o3_st(7:9); 
% Vo3 = (Po3-last_Po3)/P.Ts; 
% Ao3 = (Vo3-last_Vo3)/P.Ts;
% last_Po3 = Po3;
% last_Vo3 = Vo3;
Ro3 = o3_st(end);

Po4 = o4_st(1:3);
Vo4 = o4_st(4:6); 
Ao4 = o4_st(7:9); 
% Vo4 = (Po4-last_Po4)/P.Ts; 
% Ao4 = (Vo4-last_Vo4)/P.Ts;
% last_Po4 = Po4;
% last_Vo4 = Vo4;
Ro4 = o4_st(end);

% target
Pt = t_st(1:3); 
Vt = t_st(4:6);
At = t_st(7:9);
Rt = t_st(end);


% uav 
Pu = uav_state(1:3); 
Vu = uav_state(4:6);
Au = uav_state(7:9);
Ru = uav_state(end);

% ecbf function 
k1 = 2;
k2 = 2;
ub = [3 3 3]; % UAV velocity upper bound
lb = [-3 -3 -3]; % UAV velocity lower bound

% A = [ 1  0  0;
%      -1  0  0;
%       0  1  0;
%       0 -1  0;
%       0  0  1;
%       0  0 -1;
A = [2*(Po4(1)-Pu(1)) 2*(Po4(2)-Pu(2)) 2*(Po4(3)-Pu(3));
      2*(Po1(1)-Pu(1)) 2*(Po1(2)-Pu(2)) 2*(Po1(3)-Pu(3));
      2*(Po2(1)-Pu(1)) 2*(Po2(2)-Pu(2)) 2*(Po2(3)-Pu(3));
      2*(Po3(1)-Pu(1)) 2*(Po3(2)-Pu(2)) 2*(Po3(3)-Pu(3))];

% b = [k1*((Pt(1)+3-Pu(1))+k2*(Vt(1)-Vu(1))+At(1));
%     k1*((Pu(1)-(Pt(1)-3))+k2*(Vt(1)-Vu(1))-At(1));
%     k1*((Pt(2)+3-Pu(2))+k2*(Vt(2)-Vu(2))+At(2));
%     k1*((Pu(2)-(Pt(2)-3))+k2*(Vt(2)-Vu(2))-At(2));
%     k1*((Pt(3)+3-Pu(3))+k2*(Vt(3)-Vu(3))+At(3));
%     k1*((Pu(3)-(Pt(3)-3))+k2*(Vt(3)-Vu(3))-At(3));
b = [input_ub(Pu,Vu,Ru,Po4,Vo4,Ao4,Ro4,k1,k2);
    input_ub(Pu,Vu,Ru,Po1,Vo1,Ao1,Ro1,k1,k2);
    input_ub(Pu,Vu,Ru,Po2,Vo2,Ao2,Ro2,k1,k2);
    input_ub(Pu,Vu,Ru,Po3,Vo3,Ao3,Ro3,k1,k2)];
    

Aeq = [];
beq = [];
x0 = [0 0 0];

% cost function
f = @(x) (x(1)-acc_d(1))^2 + (x(2)-acc_d(2))^2  ;
a_new = fmincon(f,x0,A,b,Aeq,beq,lb,ub);

out = [a_new(1) a_new(2) a_new(3)];
end

function b_content = input_ub(r,v,R,r_o,v_o,a_o,R_o,k1,k2)
    b_content = k1*((r(1)-r_o(1))^2+(r(2)-r_o(2))^2+(r(3)-r_o(3))^2-(R+R_o)^2)+...
    2*k2*((r(1)-r_o(1))*(v(1)-v_o(1))+(r(2)-r_o(2))*(v(2)-v_o(2))+(r(2)-r_o(2))*(v(2)-v_o(2)))+...
    2*((v(1)-v_o(1))^2+(v(2)-v_o(2))^2+(v(3)-v_o(3))^2)-...
    2*((r(1)-r_o(1))*a_o(1)+(r(2)-r_o(2))*a_o(2)+(r(3)-r_o(3))*a_o(3));

end