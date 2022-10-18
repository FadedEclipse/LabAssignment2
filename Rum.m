function Rum(ur3,kuka,Rum_h,vertices,transformedVertices,BartransformedVertices,BenchtransformedVertices)


q1 = [1.5708   -1.5708         0   -1.5709    3.1416        -1.5708];

%KUKA TRANSPORTS RUM BOTTLE

%kuka moves from inital zeros pos to bottle home position
q1kuka = [ 0     0     0     0     0     0];
qWaypoints = [q1kuka ...
    ; [2.9671    0.2871   -0.7707         0   -0.6981         0] ...
    ; [3.1416   -0.4511   -0.1445         0   -0.6981    1.5708]];

qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));

for i = 1:size(qMatrix,1)
    kuka.model.animate(qMatrix(i,:));

    KUKASphereCollisionDetection(kuka,qMatrix(i,:),BenchtransformedVertices);
    
    drawnow();
    pause(0.01);
end

%kuka transports bottle to main bar position to put in reach of ur3
q1kuka = [3.1416   -0.4511   -0.1445         0   -0.6981    1.5708];
qWaypoints = [q1kuka ...
    ; [3.1416    0.0821   -0.5299         0   -0.4887    1.5708] ...
    ; [1.5080    0.6153   -0.5299         0    0.0978    1.5708] ...
    ; [-0.7121   -0.7793    0.4817         0   -0.2373    1.5708]];

qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));

for i = 1:size(qMatrix,1)
    kuka.model.animate(qMatrix(i,:));
    
    tr = kuka.model.fkine(qMatrix(i,:));
    transformedVertices = [vertices,ones(size(vertices,1),1)] * troty(pi/2)' * transl(-0.055,0,0)' *  tr';
    set(Rum_h,'Vertices',transformedVertices(:,1:3));
    
    KUKASphereCollisionDetection(kuka,qMatrix(i,:),BartransformedVertices);
    
    drawnow();
    pause(0.01);
end

%place bottle on main bar table
transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(-0.072,-0.662,0.55)';
set(Rum_h,'Vertices',transformedVertices(:,1:3));


%kuka moves out of area back to zeros inital pos
q1kuka = [ -0.7121   -0.7793    0.4817         0   -0.2373    1.5708];
qWaypoints = [q1kuka ...
    ; [-0.7121   -0.2461    0.1445         0   -0.2373    1.5708] ...
    ; [ 0     0     0     0     0     0]];

qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));

for i = 1:size(qMatrix,1)
    kuka.model.animate(qMatrix(i,:));
    
    KUKASphereCollisionDetection(kuka,qMatrix(i,:),BartransformedVertices);
    
    drawnow();
    pause(0.01);
end



%UR3 POURS RUM BOTTLE

%ur3 moves from inital start pos to pick up bottle
qWaypoints = [q1 ...
    ; [2.4050   -1.9478    1.5708   -1.5709    3.1416         0] ...
    ; [  2.4050   -1.0681    1.0681   -1.5709    3.1416         0]];

qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));

for i = 1:size(qMatrix,1)
    ur3.model.animate(qMatrix(i,:));
    
    UR3SphereCollisionDetection(ur3,qMatrix(i,:),BartransformedVertices)
    
    drawnow();
    pause(0.01);
end


%ur3 moves bottle to cup and starts to pour
q1 =[2.4050   -1.0681    1.0681   -1.5709    3.1416         0];
qWaypoints = [q1 ...
    ; [1.5254   -1.5708    1.5708   -1.5709    3.2673         0] ...
    ; [ 1.5254   -1.5708    1.5708   -1.5709    4.1469         0]];

qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));

for i = 1:size(qMatrix,1)
    ur3.model.animate(qMatrix(i,:));
    
    tr = ur3.model.fkine(qMatrix(i,:));
    transformedVertices = [vertices,ones(size(vertices,1),1)] * troty(pi/2)' * transl(-0.1,0,0)' * tr';
    set(Rum_h,'Vertices',transformedVertices(:,1:3));
    
    UR3SphereCollisionDetection(ur3,qMatrix(i,:),BartransformedVertices)
    
    drawnow();
    pause(0.01);
end

%pause for pouring
pause(2);

%ur3 returns bottle to table for kuka to pick up and move back home
q1 =[1.5254   -1.5708    1.5708   -1.5709    4.1469         0];
qWaypoints = [q1 ...
    ; [2.2794   -1.5708    1.5708   -1.5709    3.1416         0] ...
    ; [ 2.4050   -1.0681    1.0681   -1.5709    3.1416         0]];

qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));

for i = 1:size(qMatrix,1)
    ur3.model.animate(qMatrix(i,:));
    
    tr = ur3.model.fkine(qMatrix(i,:));
    transformedVertices = [vertices,ones(size(vertices,1),1)] * troty(pi/2)' * transl(-0.1,0,0)' * tr';
    set(Rum_h,'Vertices',transformedVertices(:,1:3));
    
    UR3SphereCollisionDetection(ur3,qMatrix(i,:),BartransformedVertices)
    
    drawnow();
    pause(0.01);
end

transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(-0.072,-0.662,0.55)';
set(Rum_h,'Vertices',transformedVertices(:,1:3));

%move ur3 out of area back to initial starting pos
q1 =[2.4050   -1.0681    1.0681   -1.5709    3.1416         0];
qWaypoints = [q1 ...
    ; [2.4050   -1.0681   -0.1885   -1.5709    3.1416         0] ...
    ; [1.5708   -1.5708         0   -1.5709    3.1416        -1.5708]];

qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));

for i = 1:size(qMatrix,1)
    ur3.model.animate(qMatrix(i,:));
    
    UR3SphereCollisionDetection(ur3,qMatrix(i,:),BartransformedVertices)
    
    drawnow();
    pause(0.01);
end

%kuka moves from inital zeros pos to bottle on main bar position
q1kuka = [ 0     0     0     0     0     0];
qWaypoints = [q1kuka ...
    ; [-0.5341   -0.4512    0.0963         0   -0.4189         0] ...
    ; [-0.7121   -0.7793    0.4817         0   -0.2373    1.5708]];

qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));

for i = 1:size(qMatrix,1)
    kuka.model.animate(qMatrix(i,:));
    
    KUKASphereCollisionDetection(kuka,qMatrix(i,:),BartransformedVertices);
    
    drawnow();
    pause(0.01);
end

%kuka transports bottle from main bar position to home position
q1kuka = [-0.7121   -0.7793    0.4817         0   -0.2373    1.5708];
qWaypoints = [q1kuka ...
    ; [1.5080    0.6153   -0.5299         0    0.0978    1.5708] ...
    ; [2.9671    0.2871   -0.7707         0   -0.6981        1.5708] ...
    ; [3.1416   -0.4511   -0.1445         0   -0.6981    1.5708]];

qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));

for i = 1:size(qMatrix,1)
    kuka.model.animate(qMatrix(i,:));
    
    tr = kuka.model.fkine(qMatrix(i,:));
    transformedVertices = [vertices,ones(size(vertices,1),1)] * troty(pi/2)' * transl(-0.055,0,0)' * tr';
    set(Rum_h,'Vertices',transformedVertices(:,1:3));
    
    KUKASphereCollisionDetection(kuka,qMatrix(i,:),BenchtransformedVertices);
    
    drawnow();
    pause(0.01);
end

%place bottle back home
transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0.5,0.7,0.5)';
set(Rum_h,'Vertices',transformedVertices(:,1:3));

%return kuka to inital zeros pos
q1kuka = [3.1416   -0.4511   -0.1445         0   -0.6981    1.5708];
qWaypoints = [q1kuka ...
    ; [2.9671    0.2871   -0.7707         0   -0.6981         0] ...
    ; [0 0 0 0 0 0]];

qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));

for i = 1:size(qMatrix,1)
    kuka.model.animate(qMatrix(i,:));
    
    KUKASphereCollisionDetection(kuka,qMatrix(i,:),BenchtransformedVertices);
    
    drawnow();
    pause(0.01);
end
end

