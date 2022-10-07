clc
clf
hold on 
clear all
camlight;
axis ([-1.25 1.25 -2 1 0 1.28]);

ur3 = UR3();

q1 = [1.5708 -1.5708 0 -1.5708 0 0];
q2 = [1.2741 -1.3963 1.6336 -1.6965 3.1416 0];
ur3.model.animate(q1);


kuka = KUKA(); %There is something wrong with this function

surf([1,1;-1,-1],[0.99,0.99;0.99,0.99],[0.5,1.28;0.5,1.28],'CData',imread('barwall.jpg'),'FaceColor','texturemap');
surf([-3,-3;+3,+3],[-3,+3;-3,+3],[0.001,0.001;0.001,0.001],'CData',imread('woodfloor.png'),'FaceColor','texturemap');
surf([1.25,1.25;-1.25,-1.25],[1,1;1,1],[0,1.28;0,1.28],'CData',imread('woodfloor.png'),'FaceColor','texturemap');
PlaceObject('bar.ply', [0,-0.7,0]);                                         %Spawn in bar
PlaceObject('Bench.ply', [0,0.7,0]);                                        %Spawn in bench
PlaceObject('safetyfence.ply', [1,0,0]);                                    %Spawn in safety fences
PlaceObject('safetyfence.ply', [-1,0,0]);
PlaceObject('UR3Box.ply',[-0.55,-0.5,0]);                                   %Spawn in UR3 box stand     
PlaceObject('SafetyButton.ply',[-1,-0.75,0.55]);                            %Spawn in safety button
PlaceObject('Cup1.ply',[-0.5,0.7,0.5]);                                     %Spawn in decorative stack of cups in the bar
PlaceObject('Cup1.ply',[-0.55,0.7,0.65]);
PlaceObject('Cup1.ply',[-0.45,0.7,0.65]);
PlaceObject('Cup1.ply',[-0.525,0.7,0.80]);
PlaceObject('Cup1.ply',[-0.475,0.7,0.80]);
PlaceObject('Cup1.ply',[-0.5,0.7,0.95]);

Sprite_h = PlaceObject('Sprite.ply');                                       %Spawn in bottle of sprite as a mixer
vertices = get(Sprite_h,'Vertices');

transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0.2,0.7,0.5)';   %Method taken from Canvas
set(Sprite_h,'Vertices',transformedVertices(:,1:3));

Coke_h = PlaceObject('Coke1.ply');                                          %Spawn in bottle of coke as a mixer
vertices = get(Coke_h,'Vertices');

transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0.3,0.7,0.5)';
set(Coke_h,'Vertices',transformedVertices(:,1:3));

Cup_h = PlaceObject('Cup1.ply');                                            %Spawn in cup
vertices = get(Cup_h,'Vertices');

transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(-0.5,-0.7,0.55)';
set(Cup_h,'Vertices',transformedVertices(:,1:3));

Cup2_h = PlaceObject('Cup1.ply');
vertices = get(Cup2_h,'Vertices');

transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(-0.65,-0.7,0.55)';
set(Cup2_h,'Vertices',transformedVertices(:,1:3));

Rum_h = PlaceObject('Rum.ply');                                             %Spawn in Rum as an alcoholic beverage
vertices = get(Rum_h,'Vertices');

transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0.5,0.7,0.5)';
set(Rum_h,'Vertices',transformedVertices(:,1:3));

Vodka_h = PlaceObject('Vodka1.ply');                                        %Spawn in Vodka as an alcoholic beverage
vertices = get(Vodka_h,'Vertices');

transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0.6,0.7,0.5)';
set(Vodka_h,'Vertices',transformedVertices(:,1:3));



ur3.model.animate(q1);
qWaypoints = [q1 ...
    ; [1.5254   -1.1450    0.6283   -0.4399         0         0] ...
    ; [1.5254   -1.1450    1.0053   -0.4399    3.1416         0] ...
    ; q2];

qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));

for i = 1:size(qMatrix,1)
ur3.model.animate(qMatrix(i,:));   

drawnow();
pause(0.01);  
end

q1 = q2;

q2 = [2.4051 -1.0193 1.1309 -1.6965 3.1416 0];

qWaypoints = [q1 ...
    ; [1.9024   -1.3963    1.6336   -1.6965    3.1416         0] ...
    ; [2.4051   -0.7680    0.8796   -1.6965    3.1416         0] ...
    ; q2];

qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));

for i = 1:size(qMatrix,1)
ur3.model.animate(qMatrix(i,:));  

drawnow();
pause(0.01);  
end




%%

clc
clf
hold on
q = [1.5708 -1.5708 0 -1.5708 0 0];
robot=UR3();
robot.model.base = transl(0,0,0);
robot.model.animate(q);
steps = 100;
a=robot.model.getpos();
pt1 = robot.model.fkine(a);
pt2 = transl(0.2,0.2,0.4); % Trapezoidal trajectory scalar

traj = ctraj(pt1,pt2,steps);
path = robot.model.ikcon(traj,zeros(1,6));

[ikPath] = RMRCTraj(robot,pt1,pt2);

for i = 1:100
robot.model.animate(ikPath(i,:));
drawnow();
pause(0.01);  
end



a=robot.model.getpos();
pt1 = robot.model.fkine(a);
pt2 = transl(-0.2,0.2,0.4); % Trapezoidal trajectory scalar

traj = ctraj(pt1,pt2,steps);
path = robot.model.ikcon(traj,zeros(1,6));

[ikPath] = RMRCTraj(robot,pt1,pt2);

for i = 1:100
robot.model.animate(ikPath(i,:));
drawnow();
pause(0.01);  
end

a=robot.model.getpos();
pt1 = robot.model.fkine(a);
pt2 = transl(-0.2,-0.2,0.4); % Trapezoidal trajectory scalar

traj = ctraj(pt1,pt2,steps);
path = robot.model.ikcon(traj,zeros(1,6));

[ikPath] = RMRCTraj(robot,pt1,pt2);

for i = 1:100
robot.model.animate(ikPath(i,:));
drawnow();
pause(0.01);  
end

a=robot.model.getpos();
pt1 = robot.model.fkine(a);
pt2 = transl(0.2,-0.2,0.4); % Trapezoidal trajectory scalar

traj = ctraj(pt1,pt2,steps);
path = robot.model.ikcon(traj,zeros(1,6));

[ikPath] = RMRCTraj(robot,pt1,pt2);

for i = 1:100
robot.model.animate(ikPath(i,:));
drawnow();
pause(0.01);  
end

