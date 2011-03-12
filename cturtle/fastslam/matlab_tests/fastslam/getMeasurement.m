function [z, H] = getMeasurement(pos, landmark_pos, observation_variance)

  vector_to_landmark = [landmark_pos(1) - pos(1);
                        landmark_pos(2) - pos(2)];

  angle_to_landmark = atan2(vector_to_landmark(2), ...
                            vector_to_landmark(1));

  landmark_distance = pdist([landmark_pos(1:2)';pos(1:2)']) + normrnd(0, observation_variance(1)*.75);

  landmark_angle = angle_to_landmark +  normrnd(0, observation_variance(2)*.7);

  q = landmark_distance^2.0;

  % Compute the measurement Jacobian
  H = [-(landmark_pos(1) - pos(1))/sqrt(q), -(landmark_pos(2) - pos(2))/sqrt(q), 0.0;
        (landmark_pos(2) - pos(2))/q,       -(landmark_pos(1) - pos(1))/q,      -1.0;
        0.0,                                0.0,                               1.0];

  z = [landmark_distance; landmark_angle; 0];
end
