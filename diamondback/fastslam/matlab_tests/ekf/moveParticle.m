function [newpos, G, V] = updateMovement(pos, movement, variance)
  speed    = normrnd(movement(1), variance(1));
  rotation = normrnd(movement(2), variance(2));

  delta = zeros(3,1);
  delta(1,1) = cos(pos(3)+rotation)*speed;
  delta(2,1) = sin(pos(3)+rotation)*speed;
  delta(3,1) = rotation;


  G = [1 0 -sin(pos(3)+rotation);
       0 1 cos(pos(3)+rotation);
       0 0 1 ];

  V = [cos(pos(3) + rotation), -sin(pos(3) + rotation);
       sin(pos(3) + rotation), cos(pos(3) + rotation);
       0                     , 1 ];


  newpos = pos+delta;
end
