function KukaCollisionDetectionExample(robot,transformedVertices)
q = [0   -0.6973         0         0         0         0];


centerPoint = [0,0,0.15];
radii = [0.1,0.1,0.3];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    robot.model.points{1} = [X(:),Y(:),Z(:)];
    warning off
    robot.model.faces{1} = delaunay(robot.model.points{1});    
    warning on;


centerPoint = [0,0,0];
radii = [0.1,0.3,0.1];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    robot.model.points{2} = [X(:),Y(:),Z(:)];
    warning off
    robot.model.faces{2} = delaunay(robot.model.points{2});    
    warning on;
    
    
centerPoint = [-0.125,0,0];
radii = [0.25,0.1,0.1];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    robot.model.points{3} = [X(:),Y(:),Z(:)];
    warning off
    robot.model.faces{3} = delaunay(robot.model.points{3});    
    warning on;

centerPoint = [0,0,0.125];
radii = [0.1,0.1,0.25];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    robot.model.points{4} = [X(:),Y(:),Z(:)];
    warning off
    robot.model.faces{4} = delaunay(robot.model.points{4});    
    warning on;    

centerPoint = [0,-0.1,0];
radii = [0.1,0.2,0.1];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    robot.model.points{5} = [X(:),Y(:),Z(:)];
    warning off
    robot.model.faces{5} = delaunay(robot.model.points{5});    
    warning on; 
    
centerPoint = [0,0,-0.15];
radii = [0.1,0.1,0.25];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    robot.model.points{6} = [X(:),Y(:),Z(:)];
    warning off
    robot.model.faces{6} = delaunay(robot.model.points{6});    
    warning on; 
    
centerPoint = [0,0,-0.15];
radii = [0.1,0.1,0.25];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    robot.model.points{7} = [X(:),Y(:),Z(:)];
    warning off
    robot.model.faces{7} = delaunay(robot.model.points{7});    
    warning on;  

robot.model.plot3d(q); 
    
tr = robot.model.fkine(q);
barPointsAndOnes = [inv(tr) * transformedVertices']';
updatedBarPoints = barPointsAndOnes(:,1:3);
algebraicDist = GetAlgebraicDist(updatedBarPoints, centerPoint, radii);
pointsInside = find(algebraicDist < 1);


display(['There are ', num2str(size(pointsInside,1)),' points inside the spheres!']);

end

