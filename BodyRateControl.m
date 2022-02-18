function m_d = BodyRateControl(pqr_d,pqr,P)

p_dot_cmd = P.kp_pqr_x * (pqr_d(1) - pqr(1));
q_dot_cmd = P.kp_pqr_y * (pqr_d(2) - pqr(2));
r_dot_cmd = P.kp_pqr_z * (pqr_d(3) - pqr(3));
m_d = [p_dot_cmd q_dot_cmd r_dot_cmd];

end