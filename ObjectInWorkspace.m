function ObjectInWorkspace(hand,v,f,fn)

hand.model.base = transl(1,0,0.6)*trotx(-pi/2)*trotz(pi);
hand.model.animate(hand.model.base);




q1 = [0];
q2 = [0];

trajectory = jtraj(q1,q2,1);


    result = IsCollision(hand,trajectory,f,v,fn);
   if result == 1
       display(['Object detected in workspace! Stop all robots!']);
   end
end

