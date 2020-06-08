function [xi] = rpy2exp(PHI,THETA,PSI)
phi = deg2rad(PHI);
theta = deg2rad(THETA);
psi = deg2rad(PSI);

Rx = [1 0 0;0 cos(phi) -sin(phi);0 sin(phi) cos(phi)];
Ry  = [cos(theta) 0 -sin(theta);0 1 0;sin(theta) 0 cos(theta)];
Rz = [cos(psi) -sin(psi) 0;sin(psi) cos(psi) 0;0 0 1];
R = Rx*Ry*Rz; 

trR = R(1,1)+R(2,2)+R(3,3);
ANGLE = acosd((trR-1)/2);
angle = acos((trR-1)/2);

w1 = (1/(2*sin(angle)))*(R(3,2)-R(2,3));
w2 = (1/(2*sin(angle)))*(R(1,3)-R(3,1));
w3 = (1/(2*sin(angle)))*(R(2,1)-R(1,2));

xi = [w1 w2 w3 ANGLE];