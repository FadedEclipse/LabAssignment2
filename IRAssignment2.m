clc
clf
hold on 
clear all
view(32,19)
camlight;
axis ([-2 2 -2 1 0 1.28]);



 ur3 = UR3(); % Call and define UR3 in workspace

q1 = [1.5708   -1.5708         0   -1.5709    3.1416        -1.5708];

ur3.model.animate(q1); % Update UR3 Position


kuka = KUKA(); % Call KUKA into workspace

hand = Hand(); % Call hand for Object in workspace function 
hand.model.base = transl(1.5,0,0.6)*trotx(-pi/2)*trotz(pi);
hand.model.animate(hand.model.base);

view(32,19) % Update view of workspace

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

[v,f,fn] = RectangularPrism([-1,-0.6,0.55], [1,0.9,1.1]); %Spawns in environments light curtains / sensors - represents a cube 


Skynet_GUI; % Runs GUI Function

%% Image Based Visual Servoing

clc 
clf 
clear all 
hold on
PlaceObject('UR3Box.ply',[-0.65,-0.05,0]);                                 %Spawn UR3 Box
bot = UR3();                                                               %Spawn UR3
bot.model.base = transl(-0.318,0.112,0.3);
qU = [0, -pi/2, deg2rad(79), deg2rad(-125), pi/2, 0];                      %Initial q values for UR3
pStar = [512;643.25];

r = KUKA();                                                                %Spawn KUKA



qK = [-pi/2; pi/4; -pi/4; 0; 0; 0];                                        %Initialq values for KUKA
bot.model.animate(qU);
r.model.animate(qK');

        m = bot.model.getpos();                                            %Set coordinates for sphere to be on end affector
        P = bot.model.fkine(m);
        P = P(1:3,4);


        
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...                     %Set up camera
'resolution', [1024 1024], 'centre', [512 512],'name', 'KUKAcamera');

% frame rate
fps = 25;

%Define values
%gain of the controler
lambda = 0.6;
%depth of the IBVS
depth = mean (P(1,:));





vel_p = [];
uv_p = [];
history = [];

 

[ikPath] = RMRCTraj(bot);                                                  %Using RMRC function to control movement path of UR3

        
        for i = 1:size(ikPath,1)
         
        Tc0= r.model.fkine(qK);


cam.T = Tc0;

% Display points in 3D and the camera
cam.plot_camera('Tcam',Tc0, 'label','scale',0.15);

p = cam.plot(P, 'Tcam', Tc0);

%camera view and plotting
cam.clf()
cam.plot(pStar, '*'); % create the camera view
cam.hold(true);
cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view

cam.hold(true);
cam.plot(P);     
        
        
        bot.model.animate(ikPath(i,:));
        
        
        m = bot.model.getpos();
        P = bot.model.fkine(m);
        P = P(1:3,4);
        sphere_h = plot_sphere(P, 0.05, 'b');                              %Plot sphere on end affector of UR3
        
        drawnow();
        pause(0.01);
        
        
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
        J2 = r.model.jacobn(qK);
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
        qM = qK + (1/fps)*qp;
        r.model.animate(qM');

        %Get camera location
        Tc = r.model.fkine(qM);
        cam.T = Tc;

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
        hist.qM = qM;

        history = [history hist];

        
        
        %update current joint position
        qK = qM;

        end
