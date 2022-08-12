landmarks = importLandmark("lm.csv");
idStart = 1;
idEnd = 1000;
close all;
pf = animatedline(0,0,0,'Color','r','Marker','.','MarkerSize',20,'LineStyle','none');
figure(1);
view(3);
axis([-100 100 -100 100 -20 100])
grid on;
for i = 1 : length(landmarks.id)
if landmarks.id(i) >= idStart && landmarks.id(i) <= idEnd 
    fprintf("Id = %d\n",landmarks.id(i));
    addpoints(pf,landmarks.x(i),landmarks.y(i),landmarks.z(i));
    drawnow limitrate
    pause(0.1)
end
end

