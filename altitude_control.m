function trust_d = altitude_control(pz_d,vz_d,az,pz,vz,R,t,P)

persistent integratedAltitudeError
persistent last_Error
if isempty(integratedAltitudeError)
    integratedAltitudeError = 0;
    last_Error = 0;
end
if P.clearpersist
    integratedAltitudeError = 0;
    last_Error = 0;
end

pos_error = pz_d - pz; %-1
integratedAltitudeError = integratedAltitudeError + pos_error*P.Ts;
pos_error_d = (pos_error-last_Error)/P.Ts;
last_Error = pos_error;
trust_d = P.mass *(P.gravity-1*pos_error-1*integratedAltitudeError-0.1*pos_error_d)/R(3,3);
trust_d = trust_d+20*vz;
trust_d = bound(trust_d,100,-100);

end