function [z, H] = getMeasurement(pos, landmark_pos, observation_variance)

  vector_to_landmark = [landmark_pos(1) - pos(1);
                        landmark_pos(2) - pos(2)];

  angle_to_landmark = atan2(vector_to_landmark(2), ...
                            vector_to_landmark(1));

  landmark_distance = sqrt((landmark_pos(1) - pos(1))^2.0 + (landmark_pos(2) - pos(2))^2.0);
  landmark_distance = landmark_distance + normrnd(0, observation_variance(1)*.25);

  landmark_angle = angle_to_landmark;
  landmark_angle = landmark_angle + normrnd(0, observation_variance(2)*.25);

%  fprintf(' Read pos(%0.3f,%0.3f) - landpos(%0.3f,%0.3f) ang: %0.3f rng:%0.3f\n',pos(1),pos(2), landmark_pos(1), landmark_pos(2), landmark_angle*180.0/pi, landmark_distance);

  q = landmark_distance^2.0;

  % Compute the measurement Jacobian
  H = [-(landmark_pos(1) - pos(1))/sqrt(q), -(landmark_pos(2) - pos(2))/sqrt(q), 0.0;
        (landmark_pos(2) - pos(2))/q,       -(landmark_pos(1) - pos(1))/q,      -1.0;
        0.0,                                0.0,                               1.0];

  z = [landmark_distance; 
       landmark_angle;
       0];
end
