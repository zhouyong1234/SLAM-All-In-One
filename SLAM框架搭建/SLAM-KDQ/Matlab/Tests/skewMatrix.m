function M = skewMatrix(a)
%SKEWMATRIX Summary of this function goes here
%   Detailed explanation goes here
M = [0,-a(3),a(2);
     a(3),0,-a(1);
     -a(2),a(1),0];
end

