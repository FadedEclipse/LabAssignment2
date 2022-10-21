function qMatrix = FineInterpolation(q1,q2,maxStepRadians)
%InterpolateWaypointRadians allows you to pass in bulk way points - 
%Within InterpolateWaypointRadians Fine interpolation function uses jtraj
%to create a minminum step where the default is 1 degree step radians -
%returns a set of poses for the robot to move through to avoid collision -
%as steps inceases in the while loop within Fine interpolation function,
%step angle decreases, the while loop continues until no pose change is
%greater than 1 degree
if nargin < 3
    maxStepRadians = deg2rad(1);
end
    
steps = 2;
while ~isempty(find(maxStepRadians < abs(diff(jtraj(q1,q2,steps))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);
end
