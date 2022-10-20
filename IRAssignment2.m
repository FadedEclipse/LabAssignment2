clc
clf
hold on 
clear all
view(32,19)
camlight;
axis ([-2 2 -2 1 0 1.28]);



 ur3 = UR3();

q1 = [1.5708   -1.5708         0   -1.5709    3.1416        -1.5708];

ur3.model.animate(q1);


kuka = KUKA(); 

view(32,19)

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

%% Image Based Visual Servoing

% Create image target (points in the image plane) 
pStar = [662 362 362 662; 362 362 662 662];

%Create 3D points
P=[1.8,1.8,1.8,1.8;
-0.25,0.25,0.25,-0.25;
 1.25,1.25,0.75,0.75];

% Add the camera
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512],'name', 'Kukacamera');

%Set FPS
fps = 25;


%Define values
%gain of the controler
lambda = 0.6;
%depth of the IBVS
depth = mean (P(1,:));


Tc0= kuka.model.fkine(qK);

drawnow

% plot camera and points
cam.T = Tc0;

% Display points in 3D and the camera
cam.plot_camera('Tcam',Tc0, 'label','scale',0.15);
plot_sphere(P, 0.02, 'b')
lighting gouraud
light

%Project points to the image
p = cam.plot(P, 'Tcam', Tc0);

%camera view and plotting
cam.clf()
cam.plot(pStar, '*'); % create the camera view
cam.hold(true);
cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view
pause(2)
cam.hold(true);
cam.plot(P);    % show initial view


%Initialise display arrays
vel_p = [];
uv_p = [];
history = [];


% loop of the visual servoing
ksteps = 0;
 while true
        ksteps = ksteps + 1;
        
        % compute the view of the camera
        uv = cam.plot(P);
        
        % compute image plane error as a column
        e = pStar-uv;   % feature error
        e = e(:);
        Zest = [];
        
        % compute the Jacobian
        if isempty(depth)
            % exact depth from simulation (not possible in practice)
            pt = homtrans(inv(Tcam), P);
            J = cam.visjac_p(uv, pt(3,:) );
        elseif ~isempty(Zest)
            J = cam.visjac_p(uv, Zest);
        else
            J = cam.visjac_p(uv, depth );
        end

        % compute the velocity of camera in camera frame
        try
            v = lambda * pinv(J) * e;
        catch
            status = -1;
            return
        end
        fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);

        %compute robot's Jacobian and inverse
        J2 = kuka.model.jacobn(qK);
        Jinv = pinv(J2);
        % get joint velocities
        qp = Jinv*v;

         
         %Maximum angular velocity cannot exceed 180 degrees/s
         ind=find(qp>pi);
         if ~isempty(ind)
             qp(ind)=pi; 
         end
         ind=find(qp<-pi);
         if ~isempty(ind)
             qp(ind)=-pi;
         end

        %Update joints 
        q = qK + (1/fps)*qp;
        kuka.model.animate(q');

        %Get camera location
        Tc = kuka.model.fkine(q);
        cam.T = Tc;
        %cam.plot_camera('Tcam',Tc, 'label','scale',0.15);
        drawnow
        
        % update the history variables
        hist.uv = uv(:);
        vel = v;
        hist.vel = vel;
        hist.e = e;
        hist.en = norm(e);
        hist.jcond = cond(J);
        hist.Tcam = Tc;
        hist.vel_p = vel;
        hist.uv_p = uv;
        hist.qp = qp;
        hist.q = q;

        history = [history hist];

         pause(1/fps)

        if ~isempty(200) && (ksteps > 200)
            break;
        end
        
        %update current joint position
        qK = q;
 end %loop finishes


            
         
%% refer to this to make ur3 move with rmrc

clc 
clear all
clf
hold on
robot = UR3();        % Load robot model

[ikPath] = RMRCTraj(robot);

for i= 1:size(ikPath,1)
robot.model.animate(ikPath(i,:));

drawnow();
pause(0.01);
end

