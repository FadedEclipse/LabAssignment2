function KUKASphereCollisionDetection(robot,q,transformedVertices)

%center points for all ellipsoids - dervied via using teach and ploting
%each ellipsoid on the robots respective links
centerPoint = [0,0,0.15; 0,0,0; -0.125,0,0; 0,0,0.125; 0,-0.1,0; 0,0,-0.15; 0,0,-0.15];

%radii for each ellipsoid in the x y and z directions - determined so that
%the robot can detect an appropriate distance for when it will come into
%collision - takes into account dimensions of each joint 
radii = [0.1,0.1,0.3; 0.1,0.3,0.1; 0.25,0.1,0.1; 0.1,0.1,0.25; 0.1,0.2,0.1; 0.1,0.1,0.25; 0.1,0.1,0.25];

%creates and plots each ellipsoid on the robots respective links using the ellipsoid function
 for i = 1:7 % one ellipsoid for each link including robot base 
a = centerPoint(i,:);
b = radii(i,:);
%Function returns ellipsoid points
[X,Y,Z] = ellipsoid( a(1), a(2), a(3), b(1), b(2), b(3) );


    robot.model.points{i} = [X(:),Y(:),Z(:)]; % set these ellipoid points, to be points on the robot 
    warning off
    robot.model.faces{i} = delaunay(robot.model.points{i});  % triangulate points to create ellipsoid face definitions   
    warning on;
end  
    
tr = zeros(4,4,robot.model.n+1);
tr(:,:,1) = robot.model.base;
L = robot.model.links;
%allows us to find which link is in collision with each link ellipsoid that
%contain points - find transfroms from base to end effector 
for i = 1 : robot.model.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha); % find location and transform of each joint and end effector in the environment
end    

% step through each joint/link and check for collisions
for i = 1: size(tr,3)    
cubePointsAndOnes = [inv(tr(:,:,i)) * transformedVertices']'; %multiple points by inverse joint transform
updatedCubePoints = cubePointsAndOnes(:,1:3); 
algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint(i,:), radii(i,:)); %check algebraic distance using function
%returns number of points within the ellipsoid 

% if algebraicDist < 1 point is inside the ellipsoid 
%if algebraicDist = 1 point is on the surface of ellispoid
%if algebraicDist >1 point is outside of ellipsoid 
pointsInside = find(algebraicDist < 1);


    display(['There are ', num2str(size(pointsInside,1)),' points inside ellipsoid ',num2str(i)]);
end       

end

