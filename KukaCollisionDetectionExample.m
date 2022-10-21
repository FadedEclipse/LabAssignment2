function KukaCollisionDetectionExample(robot,transformedVertices)
q = [0   -0.6973         0         0         0         0];
%center points for all ellipsoids - dervied via using teach and ploting
%each ellipsoid on the robots respective links

%radii for each ellipsoid in the x y and z directions - determined so that
%the robot can detect an appropriate distance for when it will come into
%collision - takes into account dimensions of each joint 

centerPoint = [0,0,0.15];
radii = [0.1,0.1,0.3];
%Function returns ellipsoid points
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    robot.model.points{1} = [X(:),Y(:),Z(:)];% set these ellipoid points, to be points on the robot 
    warning off
    robot.model.faces{1} = delaunay(robot.model.points{1});    % triangulate points to create ellipsoid face definitions  
    warning on;


centerPoint = [0,0,0];
radii = [0.1,0.3,0.1];
%Function returns ellipsoid points
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    robot.model.points{2} = [X(:),Y(:),Z(:)];% set these ellipoid points, to be points on the robot 
    warning off
    robot.model.faces{2} = delaunay(robot.model.points{2});    % triangulate points to create ellipsoid face definitions  
    warning on;
    
    
centerPoint = [-0.125,0,0];
radii = [0.25,0.1,0.1];
%Function returns ellipsoid points
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    robot.model.points{3} = [X(:),Y(:),Z(:)];% set these ellipoid points, to be points on the robot 
    warning off
    robot.model.faces{3} = delaunay(robot.model.points{3});    % triangulate points to create ellipsoid face definitions  
    warning on;

centerPoint = [0,0,0.125];
radii = [0.1,0.1,0.25];
%Function returns ellipsoid points
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    robot.model.points{4} = [X(:),Y(:),Z(:)];% set these ellipoid points, to be points on the robot 
    warning off
    robot.model.faces{4} = delaunay(robot.model.points{4});  % triangulate points to create ellipsoid face definitions    
    warning on;    

centerPoint = [0,-0.1,0];
radii = [0.1,0.2,0.1];
%Function returns ellipsoid points
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    robot.model.points{5} = [X(:),Y(:),Z(:)];% set these ellipoid points, to be points on the robot 
    warning off
    robot.model.faces{5} = delaunay(robot.model.points{5});    % triangulate points to create ellipsoid face definitions  
    warning on; 
    
centerPoint = [0,0,-0.15];
radii = [0.1,0.1,0.25];
%Function returns ellipsoid points
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    robot.model.points{6} = [X(:),Y(:),Z(:)];% set these ellipoid points, to be points on the robot 
    warning off
    robot.model.faces{6} = delaunay(robot.model.points{6});    % triangulate points to create ellipsoid face definitions  
    warning on; 
    
centerPoint = [0,0,-0.15];
radii = [0.1,0.1,0.25];
%Function returns ellipsoid points
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

    robot.model.points{7} = [X(:),Y(:),Z(:)];% set these ellipoid points, to be points on the robot 
    warning off
    robot.model.faces{7} = delaunay(robot.model.points{7});    % triangulate points to create ellipsoid face definitions  
    warning on;  

robot.model.plot3d(q); 
% step through each joint/link and check for collisions    
tr = robot.model.fkine(q);
barPointsAndOnes = [inv(tr) * transformedVertices']'; %multiple points by inverse joint transform
updatedBarPoints = barPointsAndOnes(:,1:3);
algebraicDist = GetAlgebraicDist(updatedBarPoints, centerPoint, radii);  %check algebraic distance using function
%returns number of points within the ellipsoid 

% if algebraicDist < 1 point is inside the ellipsoid 
%if algebraicDist = 1 point is on the surface of ellispoid
%if algebraicDist >1 point is outside of ellipsoid 
pointsInside = find(algebraicDist < 1);


display(['There are ', num2str(size(pointsInside,1)),' points inside the spheres!']);

end

