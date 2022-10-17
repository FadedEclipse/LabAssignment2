function UR3SphereCollisionDetection(robot,q,transformedVertices)

centerPoint = [0,0,0.05];
radii = [0.05,0.05,0.1];
[X,Y,Z] = ellipsoid(centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    robot.model.points{1} = [X(:),Y(:),Z(:)];
    warning off
    robot.model.faces{1} = delaunay(robot.model.points{1});    
    warning on;


centerPoint = [0,0,0.075];
radii = [0.05,0.05,0.15];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    robot.model.points{2} = [X(:),Y(:),Z(:)];
    warning off
    robot.model.faces{2} = delaunay(robot.model.points{2});    
    warning on;
    
    
centerPoint = [0.09,0,0.121825];
radii = [0.18,0.05,0.05];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    robot.model.points{3} = [X(:),Y(:),Z(:)];
    warning off
    robot.model.faces{3} = delaunay(robot.model.points{3});    
    warning on;

centerPoint = [0.1,0,0.03];
radii = [0.18,0.05,0.05];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    robot.model.points{4} = [X(:),Y(:),Z(:)];
    warning off
    robot.model.faces{4} = delaunay(robot.model.points{4});    
    warning on;    

centerPoint = [0,0,0];
radii = [0.05,0.1,0.05];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    robot.model.points{5} = [X(:),Y(:),Z(:)];
    warning off
    robot.model.faces{5} = delaunay(robot.model.points{5});    
    warning on; 
    
centerPoint = [0,0,0];
radii = [0.05,0.05,0.1];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    robot.model.points{6} = [X(:),Y(:),Z(:)];
    warning off
    robot.model.faces{6} = delaunay(robot.model.points{6});    
    warning on; 
    
centerPoint = [0,0,-0.05];
radii = [0.05,0.05,0.1];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    robot.model.points{7} = [X(:),Y(:),Z(:)];
    warning off
    robot.model.faces{7} = delaunay(robot.model.points{7});    
    warning on;  
    
  

tr = robot.model.fkine(q);
cubePointsAndOnes = [inv(tr) * transformedVertices']';
updatedCubePoints = cubePointsAndOnes(:,1:3);
algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
pointsInside = find(algebraicDist < 1);


    display(['There are ', num2str(size(pointsInside,1)),' points inside the spheres!']);
end

