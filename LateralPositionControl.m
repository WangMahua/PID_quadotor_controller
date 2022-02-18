function acc_d = LateralPositionControl(pos_d,vel_d,pos,vel,acc_,P)
acc_(3) = 0;
vel_d(3) = 0;
pos_d(3) = pos(3);
acc_d = acc_;

persistent integratedposError 
persistent last_pos_error
if isempty(integratedposError)
    integratedposError = [0 0 0];
    last_pos_error = [0 0 0];
end
if P.clearpersist
    integratedposError = [0 0 0];
    last_pos_error = [0 0 0];
end

pos_error = pos_d-pos; %1
pos_error_for_d = (pos_error-last_pos_error)/P.Ts; %1
integratedposError = integratedposError + pos_error*P.Ts;
%acc_d = 0.6 * (pos_d-pos) + 0.002*pos_error_for_d + 10*integratedposError;
%acc_d = acc_d+2.45 * (- vel) ;
%acc_d = 1 * (pos_d-pos) + 0.002*pos_error_for_d + 1*integratedposError;
acc_d = 1 * (pos_d-pos) + 0.002*pos_error_for_d + 30*integratedposError;
acc_d = 5*acc_d+3 * (- vel) ;

% vel acc limitation 
% if norm(vel_d) > P.max_vel_xy 
%     vel_d = (vel_d/norm(vel_d))*P.max_vel_xy ;
% end

% if norm(acc_d) > P.max_acc_xy 
%     acc_d = (acc_d/norm(acc_d))*P.max_acc_xy ;
% end

last_pos_error = pos_error;
end 