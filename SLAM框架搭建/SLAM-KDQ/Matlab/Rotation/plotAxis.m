function plotAxis(origin,pointer,bold)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
x0 = origin(1);
y0 = origin(2);
z0 = origin(3);
len = length(pointer);
for i = 1 : len 
p = pointer(:,i);
x1 = p(1);
y1 = p(2);
z1 = p(3);
if i == 1
    if nargin == 3 && bold == 1 
        quiver3(x0,y0,z0,x1,y1,z1,'LineWidth',2,'Color','r');
    else
        quiver3(x0,y0,z0,x1,y1,z1,'-r');
    end
xlabel('x');
hold on 
elseif i == 2
    if nargin == 3 && bold == 1 
        quiver3(x0,y0,z0,x1,y1,z1,'LineWidth',2,'Color','g');
    else
        quiver3(x0,y0,z0,x1,y1,z1,'-g');
    end
ylabel('y');
hold on;
else
    if nargin == 3 && bold == 1 
        quiver3(x0,y0,z0,x1,y1,z1,'LineWidth',2,'Color','b');
    else
        quiver3(x0,y0,z0,x1,y1,z1,'-b');
    end
zlabel('z');
hold on;
end
grid on;
axis([-3 3 -3 3 -3 3]);
hold on;

end


