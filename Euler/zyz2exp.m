function [xi] = zyz2exp(ALPHA,BETA,THETA)
alpha = deg2rad(ALPHA);
beta = deg2rad(BETA);
theta = deg2rad(THETA);
Rz1 = [cos(alpha) -sin(alpha) 0;sin(alpha) cos(alpha) 0;0 0 1];
Ry  = [cos(beta) 0 -sin(beta);0 1 0;sin(beta) 0 cos(beta)];
Rz2 = [cos(theta) -sin(theta) 0;sin(theta) cos(theta) 0;0 0 1];

R = Rz1*Ry*Rz2;

angle = acos((R(1,1)+R(2,2)+R(3,3)-1)/2);
ANGLE = acosd((R(1,1)+R(2,2)+R(3,3)-1)/2);

w1 = (1/(2*sin(angle)))*(R(3,2)-R(2,3));
w2 = (1/(2*sin(angle)))*(R(1,3)-R(3,1));
w3 = (1/(2*sin(angle)))*(R(2,1)-R(1,2));

xi = [w1 w2 w3 ANGLE];
