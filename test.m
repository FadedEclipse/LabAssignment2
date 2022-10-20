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
        
        
       

