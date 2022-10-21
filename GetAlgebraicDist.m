function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)

% if algebraicDist < 1 point is inside the ellipsoid 
%if algebraicDist = 1 point is on the surface of ellispoid
%if algebraicDist >1 point is outside of ellipsoid 

%The general equation for a sphere in 3D space is given as:
algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
              + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
              + ((points(:,3)-centerPoint(3))/radii(3)).^2;
end

