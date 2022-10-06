function [Phi] = WrapperQuat2Angle(q)
[psi, theta, phi] = quat2angle(q');

Phi = [phi; theta; psi];