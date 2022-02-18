function pqr_d = RollPitchControl(a_d,R,thrust_d,P)

pqr_d = [0 0 0];

c = -thrust_d/P.mass;
b_x = bound(a_d(1)/c,sin(pi/12),-sin(pi/12));
b_y = bound(a_d(2)/c,sin(pi/12),-sin(pi/12));
%pqr_d(1) = b_y;
%pqr_d(2) = b_x;
b_x_dot = P.kp_bank *(b_x-R(1,3));
b_y_dot = P.kp_bank *(b_y-R(2,3));
pqr_d(1)= (R(2,1)*b_x_dot-R(1,1)*b_y_dot)/R(3,3);
pqr_d(2)= (R(2,2)*b_x_dot-R(1,2)*b_y_dot)/R(3,3);
end