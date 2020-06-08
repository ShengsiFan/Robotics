function [angles] = exp2zyz(W1,W2,W3,THETA)
%brief transforms rigid transformation matrices from the exponential representation to the RPY convention
%the norm of vector w,and this step is to unitize vector w
norm = sqrt(W1^2+W2^2+W3^2);
w1 = W1/norm;
w2 = W2/norm;
w3 = W3/norm;
%alpha is the rotational angle about w
theta = deg2rad(THETA);
%alpha beta gamma are rotational angles about x,y,z
BETA = acos((w3^2)*(1-cos(theta))+cos(theta));
beta = acosd((w3^2)*(1-cos(theta))+cos(theta));
if beta == 0
    angles = [THETA 0 0];
else
    alpha = asind(-(w2*w3*(1-cos(theta))-w1*sin(theta))/(sin(BETA)));
    gamma = asind(-(w2*w3*(1-cos(theta))+w1*sin(theta))/(sin(BETA)));
    angles = [alpha beta gamma]; %the output vector containing the angles about x y z
end