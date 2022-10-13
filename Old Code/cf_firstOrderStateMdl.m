function [X_new] = cf_firstOrderStateMdl(X, U)


%   Note, X as a state vector is X as a 
p = X(1); q = X(2); r = X(3); phi = X(4); theta = X(5); psi = X(6);
ub = X(7); vb = X(8); wb = X(9); x = X(10); y = X(11); z = X(11);

rot

T = U(1); tauX = U(2); tauY = U(3); tauZ = U(4);
g = 9.81;

F_gravity_inertialFrame = [0, 0, -mg];
omega_inert2body = [p, q, r]';




%% Defining Constants
Ixx= 1.43e-5; Iyy = 1.43e-5; Izz = 2.89e-5; %[kgm^2]
d = 0.046; %[m] distance from center of mass to rotor
J_matrix = diag([Ixx, Iyy, Izz]); % Inertia Matrix about body axis
k_M = 1.5e-9; %[Nm/rpm^2]
k_f = 6.11e-8;     %[N/rpm^2]
k_motor = 20; %[1/second]
m = 0.0030; %[kg]
RotMatrix = "ZYX"; %Order of rotations
drag = [0.1, 0.1, .5]; %[kg/s]

M_p1 = J_matrix*omega_inert2body;
M_p2 = cross(omega_inert2body, (J_matrix*omega_inert2body));






state_vec_confirmed = [-g*sin(theta) + r*v - q*w;
    g*sin(phi)*cos(theta) - r*u + p*w;
    1 / m *(-Fz) + g*cos(phi)*cos(theta) + q*u - p*v;
    (1 / Ixx) * (L + (Iyy - Izz)*q*r);
    (1/ Iyy) *(M + (Izz-Ixx)*p*r);
    (1/ Izz) *(N + (Izz - Iyy)*p*q);
    p + (q*sin(phi) + r*cos(phi))*tan(theta);
    q*cos(phi) - r*sin(phi);
    (q*sin(phi) + r*cos(phi))*sec(theta);]

end