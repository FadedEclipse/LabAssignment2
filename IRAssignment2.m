clc
clf
hold on 
clear all
axis ([-1.25 1.25 -2 1 0 1.28]);

robot = UR3();

initialState = [1.5708 -1.5708 0 -1.5708 0 0];
robot.model.animate(initialState);


kuka = KUKA();

surf([1,1;-1,-1],[0.99,0.99;0.99,0.99],[0.5,1.28;0.5,1.28],'CData',imread('barwall.jpg'),'FaceColor','texturemap');
surf([-3,-3;+3,+3],[-3,+3;-3,+3],[0.001,0.001;0.001,0.001],'CData',imread('woodfloor.png'),'FaceColor','texturemap');
surf([1.25,1.25;-1.25,-1.25],[1,1;1,1],[0,1.28;0,1.28],'CData',imread('woodfloor.png'),'FaceColor','texturemap');
PlaceObject('bar.ply', [0,-0.7,0]);
PlaceObject('Bench.ply', [0,0.7,0]);
PlaceObject('safetyfence.ply', [1,0,0]);
PlaceObject('safetyfence.ply', [-1,0,0]);
