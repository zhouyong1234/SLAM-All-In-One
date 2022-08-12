function skew = vec3d2SkewMat(rotVec)
%VEC3D2ROTMAT 此处显示有关此函数的摘要
%   此处显示详细说明
x = rotVec(1);
y = rotVec(2);
z = rotVec(3);
skew = [0,-z,y;z,0,-x;-y,x,0];

end

