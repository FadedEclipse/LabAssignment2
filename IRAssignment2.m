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
surf([2,2;-2,-2],[1,1;1,1],[0,1.28;0,1.28],'CData',imread('woodfloor.png'),'FaceColor','texturemap');                                        %Spawn in bar
                                       %Spawn in bench
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


Bar = PlaceObject('bar.ply'); 
vertices = get(Bar,'Vertices');

BartransformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0,-0.7,0)';   %Method taken from Canvas
set(Bar,'Vertices',BartransformedVertices(:,1:3));

Bench = PlaceObject('Bench.ply'); 
vertices = get(Bench,'Vertices');

BenchtransformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0,0.7,0)';   %Method taken from Canvas
set(Bench,'Vertices',BenchtransformedVertices(:,1:3));

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



Skynet_GUI;



