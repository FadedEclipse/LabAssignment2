function ObjectOutOfWorkspace(hand)
hand.model.base = transl(1.5,0,0.6)*trotx(-pi/2)*trotz(pi);
hand.model.animate(hand.model.base);
end

