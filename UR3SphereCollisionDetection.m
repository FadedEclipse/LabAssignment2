function UR3SphereCollisionDetection(robot,q,transformedVertices)


centerPoint = [0,0,0.05; 0,0,0.075; 0.09,0,0.121825; 0.1,0,0.03; 0,0,0; 0,0,0; 0,0,-0.05];

radii = [0.05,0.05,0.1; 0.05,0.05,0.15; 0.18,0.05,0.05; 0.18,0.05,0.05; 0.05,0.1,0.05; 0.05,0.05,0.1; 0.05,0.05,0.1];

 for i = 1:7
a = centerPoint(i,:);
b = radii(i,:);
[X,Y,Z] = ellipsoid( a(1), a(2), a(3), b(1), b(2), b(3) );


    robot.model.points{i} = [X(:),Y(:),Z(:)];
    warning off
    robot.model.faces{i} = delaunay(robot.model.points{i});    
    warning on;
end  
    
tr = zeros(4,4,robot.model.n+1);
tr(:,:,1) = robot.model.base;
L = robot.model.links;
for i = 1 : robot.model.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end    
    
for i = 1: size(tr,3)    
cubePointsAndOnes = [inv(tr(:,:,i)) * transformedVertices']';
updatedCubePoints = cubePointsAndOnes(:,1:3);
algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint(i,:), radii(i,:));
pointsInside = find(algebraicDist < 1);


    display(['There are ', num2str(size(pointsInside,1)),' points inside ellipsoid ',num2str(i)]);
end 

end

