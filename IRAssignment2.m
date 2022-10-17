clc
clf
hold on 
clear all
camlight;

axis ([-2 2 -2 1 0 1.28]);



 ur3 = UR3();

q1 = [1.5708   -1.5708         0   -1.5709    3.1416        -1.5708];
ur3.model.animate(q1);


kuka = KUKA(); 



surf([1,1;-1,-1],[0.99,0.99;0.99,0.99],[0.5,1.28;0.5,1.28],'CData',imread('barwall.jpg'),'FaceColor','texturemap');
surf([-3,-3;+3,+3],[-3,+3;-3,+3],[0.001,0.001;0.001,0.001],'CData',imread('woodfloor.png'),'FaceColor','texturemap');
surf([2,2;-2,-2],[1,1;1,1],[0,1.28;0,1.28],'CData',imread('woodfloor.png'),'FaceColor','texturemap');
PlaceObject('bar.ply', [0,-0.7,0]);                                         %Spawn in bar
PlaceObject('Bench.ply', [0,0.7,0]);                                        %Spawn in bench
PlaceObject('safetyfence.ply', [1,0,0]);                                    %Spawn in safety fences
PlaceObject('safetyfence.ply', [-1,0,0]);
PlaceObject('UR3Box.ply',[-0.55,-0.5,0]);                                   %Spawn in UR3 box stand     
PlaceObject('SafetyButton.ply',[-1,-0.75,0.55]);                            %Spawn in safety button
PlaceObject('Cup1.ply',[-0.5,0.7,0.5]);                                     %Spawn in decorative stack of cups in the bar
PlaceObject('Cup1.ply',[-0.6,0.7,0.5]);
PlaceObject('Cup1.ply',[-0.4,0.7,0.5]);
PlaceObject('Cup1.ply',[-0.55,0.7,0.65]);
PlaceObject('Cup1.ply',[-0.45,0.7,0.65]);
PlaceObject('Cup1.ply',[-0.5,0.7,0.8]);
PlaceObject('aid.ply',[1.35,0.6,0]);
PlaceObject('fire.ply',[-1.45,0.8,0]);
PlaceObject('sign1.ply',[1.04,0,0.5]);
PlaceObject('sign.ply',[-1.04,0,0.5]);
PlaceObject('seccy.ply',[-1.55,-0.85,0]);


Sprite_h = PlaceObject('Sprite.ply');                                       %Spawn in bottle of sprite as a mixer
vertices = get(Sprite_h,'Vertices');

transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0.2,0.7,0.5)';   %Method taken from Canvas
set(Sprite_h,'Vertices',transformedVertices(:,1:3));

Coke_h = PlaceObject('Coke1.ply');                                          %Spawn in bottle of coke as a mixer
vertices = get(Coke_h,'Vertices');

transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0.3,0.7,0.5)';
set(Coke_h,'Vertices',transformedVertices(:,1:3));

drink_h = PlaceObject('cocktailshaker.ply');                                        %Spawn in Vodka as an alcoholic beverage
vertices = get(drink_h ,'Vertices');

transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(-0.5,-0.7,0.55)';
set(drink_h,'Vertices',transformedVertices(:,1:3));

Vodka_h = PlaceObject('Vodka1.ply');                                        %Spawn in Vodka as an alcoholic beverage
vertices = get(Vodka_h,'Vertices');

transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0.6,0.7,0.5)';
set(Vodka_h,'Vertices',transformedVertices(:,1:3));

Rum_h = PlaceObject('Rum.ply');                                             %Spawn in Rum as an alcoholic beverage
vertices = get(Rum_h,'Vertices');

transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0.5,0.7,0.5)';
set(Rum_h,'Vertices',transformedVertices(:,1:3));



%Skynet_GUI;

%%
clc
clf
hold on 
clear all
camlight;



kuka = KUKA();

q = [ -0.6283   -0.7383   -0.1927    0.0646         0         0];


centerPoint = [0,0,0.15];
radii = [0.1,0.1,0.3];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    kuka.model.points{1} = [X(:),Y(:),Z(:)];
    warning off
    kuka.model.faces{1} = delaunay(kuka.model.points{1});    
    warning on;


centerPoint = [0,0,0];
radii = [0.1,0.3,0.1];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    kuka.model.points{2} = [X(:),Y(:),Z(:)];
    warning off
    kuka.model.faces{2} = delaunay(kuka.model.points{2});    
    warning on;
    
    
centerPoint = [-0.125,0,0];
radii = [0.25,0.1,0.1];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    kuka.model.points{3} = [X(:),Y(:),Z(:)];
    warning off
    kuka.model.faces{3} = delaunay(kuka.model.points{3});    
    warning on;

centerPoint = [0,0,0.125];
radii = [0.1,0.1,0.25];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    kuka.model.points{4} = [X(:),Y(:),Z(:)];
    warning off
    kuka.model.faces{4} = delaunay(kuka.model.points{4});    
    warning on;    

centerPoint = [0,-0.1,0];
radii = [0.1,0.2,0.1];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    kuka.model.points{5} = [X(:),Y(:),Z(:)];
    warning off
    kuka.model.faces{5} = delaunay(kuka.model.points{5});    
    warning on; 
    
centerPoint = [0,0,-0.125];
radii = [0.1,0.1,0.25];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    kuka.model.points{6} = [X(:),Y(:),Z(:)];
    warning off
    kuka.model.faces{6} = delaunay(kuka.model.points{6});    
    warning on; 
    
centerPoint = [0,0,-0.125];
radii = [0.1,0.1,0.25];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    kuka.model.points{7} = [X(:),Y(:),Z(:)];
    warning off
    kuka.model.faces{7} = delaunay(kuka.model.points{7});    
    warning on;  
    

axis equal
camlight

kuka.model.plot3d(zeros(1,6));

Bar = PlaceObject('bar.ply'); 
vertices = get(Bar,'Vertices');

transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0,-0.7,0)';   %Method taken from Canvas
set(Bar,'Vertices',transformedVertices(:,1:3));



% 
% q = [ 0.0000   -1.1894    0.2890   -0.0646    0.5864         0];
% tr = kuka.model.fkine(q);
% cubePointsAndOnes = [inv(tr) * transformedVertices']';
% updatedCubePoints = cubePointsAndOnes(:,1:3);
% algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
% pointsInside = find(algebraicDist < 1);
% display(['2.9: There are now ', num2str(size(pointsInside,1)),' points inside']);

