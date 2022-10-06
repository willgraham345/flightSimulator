function [u2] = attitudeCtrl(J, k_r, k_omega, e_r, e_omega, Omega, DCM, Omega_des, DCM_des)
p1 = J*(-k_r*e_r - k_omega*e_omega)

p2 = cross(Omega, J*Omega)
Omega_skew = [0, -Omega(3) Omega(2);
    Omega(3), 0, -Omega(1);
    -Omega(2), Omega(1),  0];

p3 = J*(Omega_skew*DCM'*DCM_des*Omega_des - DCM'*DCM_des*)

end