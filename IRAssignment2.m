clc
clf
hold on 
clear all
camlight;
axis ([-1.25 1.25 -2 1 0 1.28]);

ur3 = UR3();

initialState = [1.5708 -1.5708 0 -1.5708 0 0];
ur3.model.animate(initialState);


kuka = KUKA();

surf([1,1;-1,-1],[0.99,0.99;0.99,0.99],[0.5,1.28;0.5,1.28],'CData',imread('barwall.jpg'),'FaceColor','texturemap');
surf([-3,-3;+3,+3],[-3,+3;-3,+3],[0.001,0.001;0.001,0.001],'CData',imread('woodfloor.png'),'FaceColor','texturemap');
surf([1.25,1.25;-1.25,-1.25],[1,1;1,1],[0,1.28;0,1.28],'CData',imread('woodfloor.png'),'FaceColor','texturemap');
PlaceObject('bar.ply', [0,-0.7,0]);
PlaceObject('Bench.ply', [0,0.7,0]);
PlaceObject('safetyfence.ply', [1,0,0]);
PlaceObject('safetyfence.ply', [-1,0,0]);
PlaceObject('UR3Box.ply',[-0.55,-0.5,0]);
PlaceObject('SafetyButton.ply',[-1,-0.75,0.55]);

Sprite_h = PlaceObject('Sprite.ply');
vertices = get(Sprite_h,'Vertices');

transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0.2,0.7,0.5)';
set(Sprite_h,'Vertices',transformedVertices(:,1:3));

Coke_h = PlaceObject('Coke1.ply');
vertices = get(Coke_h,'Vertices');

transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0.3,0.7,0.5)';
set(Coke_h,'Vertices',transformedVertices(:,1:3));

Cup_h = PlaceObject('Cup1.ply');
vertices = get(Cup_h,'Vertices');

transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(-0.5,-0.7,0.55)';
set(Cup_h,'Vertices',transformedVertices(:,1:3));

Cup2_h = PlaceObject('Cup1.ply');
vertices = get(Cup2_h,'Vertices');

transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(-0.65,-0.7,0.55)';
set(Cup2_h,'Vertices',transformedVertices(:,1:3));

Rum_h = PlaceObject('Rum.ply');
vertices = get(Rum_h,'Vertices');

transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0.5,0.7,0.5)';
set(Rum_h,'Vertices',transformedVertices(:,1:3));

Vodka_h = PlaceObject('Vodka1.ply');
vertices = get(Vodka_h,'Vertices');

transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0.6,0.7,0.5)';
set(Vodka_h,'Vertices',transformedVertices(:,1:3));



%%

clc
clf
robot=UR5();
pt1 = transl(0.5,-0.3,0.4);
pt2 = transl(0.5,0.3,0.4); % Trapezoidal trajectory scalar

RMRCTraj(robot,pt1,pt2);
