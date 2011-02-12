function [z, H] = getMeasurement(pos, landmark_pos, observation_variance)

  vector_to_landmark = [landmark_pos(1) - pos(1);
                        landmark_pos(2) - pos(2)];

  angle_to_landmark = atan2(vector_to_landmark(2), ...
                            vector_to_landmark(1));

  landmark_distance = ...
    normrnd(pdist([landmark_pos';pos(1:2)']), observation_variance(1));

  landmark_angle = normrnd(angle_to_landmark - pos(3), observation_variance(2));

  q = landmark_distance^2;

  H = [ -(landmark_pos(1) - pos(1))/sqrt(q), -(landmark_pos(2) - pos(2))/sqrt(q), 0;
         (landmark_pos(2) - pos(2))/q,       -(landmark_pos(1) - pos(1))/q,      -1;
         0, 0, 0];

  z = [landmark_distance; landmark_angle; 0];
end
