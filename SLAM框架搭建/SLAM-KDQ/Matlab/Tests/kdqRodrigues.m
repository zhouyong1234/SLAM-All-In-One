function R = kdqRodrigues(u,theta)
%ROD Summary of this function goes here
%   Detailed explanation goes here
u = u/norm(u);
skewu = skewMatrix(u);
I = [1,0,0;0,1,0;0,0,1];
R = I + skewu * sin(theta) + skewu * skewu * (1 - cos(theta))
end

