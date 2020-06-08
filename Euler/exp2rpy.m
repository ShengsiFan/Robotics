function [angles] = exp2rpy(W1,W2,W3,ALPHA)
norm = sqrt(W1^2+W2^2+W3^2);
w1 = W1/norm;
w2 = W2/norm;
w3 = W3/norm;
alpha = deg2rad(ALPHA);
theta = asind(w1*w3*(1-cos(alpha))+w2*sin(alpha));
THETA = asin(w1*w3*(1-cos(alpha))+w2*sin(alpha));
psi = acosd((cos(alpha)+(1-cos(alpha))*(w1^2))/cos(THETA));
phi = acosd((cos(alpha)+(1-cos(alpha))*(w3^2))/cos(THETA));
angles = [phi theta psi];

