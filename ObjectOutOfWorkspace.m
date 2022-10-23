function ObjectOutOfWorkspace(hand)
%this function simply moves the hand out of the light curtain 
hand.model.base = transl(1.5,0,0.6)*trotx(-pi/2)*trotz(pi);
hand.model.animate(hand.model.base);
end

