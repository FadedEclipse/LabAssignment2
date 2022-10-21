function qMatrix = InterpolateWaypointRadians(waypointRadians,maxStepRadians)
%InterpolateWaypointRadians allows you to pass in bulk way points - 
%Within InterpolateWaypointRadians Fine interpolation function uses jtraj
%to create a minminum step where the default is 1 degree step radians -
%returns a set of poses for the robot to move through to avoid collision -
%as steps inceases in the while loop within Fine interpolation function,
%step angle decreases, the while loop continues until no pose change is
%greater than 1 degree
if nargin < 2
    maxStepRadians = deg2rad(1);
end

qMatrix = [];
for i = 1: size(waypointRadians,1)-1
    qMatrix = [qMatrix ; FineInterpolation(waypointRadians(i,:),waypointRadians(i+1,:),maxStepRadians)]; %#ok<AGROW>
end
end
