 bot = UR3();
 qU = [0; -pi/2; deg2rad(79); deg2rad(-125); pi/2; 0;];
bot.model.base = transl(-0.5,0.112,0.3);

bot.model.animate(qU');
drawnow

m = bot.model.getpos();
P = bot.model.fkine(m);
P = P(1:3,4);

plot_sphere(P, 0.05, 'b')

a = transl([-0.318,-0.512,1.05]);
        startPT = bot.model.fkine(qU);
        endPT = a;
        tPath = ctraj(startPT, endPT, 50);
        
        for i = 1:50
        ikinePath = bot.model.ikcon(tPath, qU);
        
        bot.model.animate(ikinePath(i,:));
        
        %drawnow();
        
        %qU = ikinePath(end,:);
        
        m = bot.model.getpos();
        P = bot.model.fkine(m);
        P = P(1:3,4);
        plot_sphere(P, 0.05, 'b')
        drawnow();
        pause(0.1);
        end
        
        qU = ikinePath(end,:);