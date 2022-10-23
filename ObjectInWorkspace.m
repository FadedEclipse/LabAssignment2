function ObjectInWorkspace(hand,v,f,fn)
%Move hand object into collision of the light curtain
hand.model.base = transl(1,0,0.6)*trotx(-pi/2)*trotz(pi);
hand.model.animate(hand.model.base);



%create traj with no movements 
q1 = [0];
q2 = [0];

trajectory = jtraj(q1,q2,1);

%send robot hand - trajectory and faces verticies and face normals to is
%collision function - uses line plane intersection to detect if an object
%is within the robots workspace - if there is a collision the function will
%return a result of 1 and print the warning message to the command window
%and stop robot operations until the object has been removed
result = IsCollision(hand,trajectory,f,v,fn);
   if result == 1
       display(['Object detected in workspace! Stop all robots!']);
   end
end

