clc 
clf 
clear all 
hold on
bot = UR3();
bot.model.base = transl(-0.318,0.112,0.3);
qU = [0, -pi/2, deg2rad(79), deg2rad(-125), pi/2, 0];
pStar = [512;512];

r = KUKA();



qK = [-pi/2; pi/4; -pi/4; 0; 0; 0];
bot.model.animate(qU);
r.model.animate(qK');

m = bot.model.getpos();
        P = bot.model.fkine(m);
        P = P(1:3,4);

cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512],'name', 'KUKAcamera');

% frame rate
fps = 25;

%Define values
%gain of the controler
lambda = 0.6;
%depth of the IBVS
depth = mean (P(1,:));

Tc0= r.model.fkine(qK);
drawnow

cam.T = Tc0;

% Display points in 3D and the camera
cam.plot_camera('Tcam',Tc0, 'label','scale',0.15);

p = cam.plot(P, 'Tcam', Tc0);

%camera view and plotting
cam.clf()
cam.plot(pStar, '*'); % create the camera view
cam.hold(true);
cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view
pause(2)
cam.hold(true);
cam.plot(P); 

vel_p = [];
uv_p = [];
history = [];


        
      

[ikPath] = RMRCTraj(bot);
        
        for i = 1:size(ikPath,1)
        
        
        bot.model.animate(ikPath(i,:));
        
        
        m = bot.model.getpos();
        P = bot.model.fkine(m);
        P = P(1:3,4);
        sphere_h = plot_sphere(P, 0.05, 'b');
        
        drawnow();
        pause(0.1);
        delete(sphere_h);
        
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

         pause(1/fps)

        
        %update current joint position
        qK = qM;

        end
        
        
 %% 
 
 clc 
clf 
clear all 
hold on
bot = UR3();
qU = [0, -pi/2, deg2rad(79), deg2rad(-125), pi/2, 0];

bot.model.animate(qU);


[ikPath] = RMRCTraj(bot);
        
        for i = 1:size(ikPath,1)
        
        
        bot.model.animate(ikPath(i,:));
        
        
        m = bot.model.getpos();
        P = bot.model.fkine(m);
        P = P(1:3,4);
        sphere_h = plot_sphere(P, 0.05, 'b');
        
        drawnow();
        pause(0.1);
        delete(sphere_h);
        end

