timesteps = 60;
arrow_length = .5;
max_read_distance = 9.5;


%%%%%%%%%%%%%%%%  l1  l2  l3
real_landmarks = [1.0,  2.0,  0.0;     % x
                  2.0,  1.0   2.0;     % y
                  0.0,  0.0   0.0];    % Nuthin

real_position = [0.0;     % x
                 0.0;     % y
                 pi/4.0]; % rotation

movement_command = [.07;     % Distance
                    .03];    % Rotation
                    
movement_variance = [.05;   % Distance
                     .035]; % Rotation

M = [movement_variance(1), 0.0;
     0.0, movement_variance(2)];

measurement_variance = [0.01; % Distance
                        .001; % Angle
                        0]; % Nuthin

R = [measurement_variance(1), 0.0, 0.0;
     0.0, measurement_variance(2), 0.0;
     0.0, 0.0, measurement_variance(3)];


% Create the particles 
num_particles = 300;
for i = 1:num_particles

  particles(i).w = 1.0/num_particles;

  particles(i).position = real_position;

  for lIdx=1:length(real_landmarks)
    particles(i).landmarks(lIdx).seen = false;
  end

end

pos_history = [];

for timestep = 1:timesteps
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Move the actual robot
  real_position = moveParticle(real_position, movement_command, movement_variance);
  pos_history = [pos_history, real_position];

  % Move the actual particles
  for pIdx = 1:num_particles
    [particles(pIdx).position,G_p, V_p] = moveParticle( ...
        particles(pIdx).position, movement_command, movement_variance);
  end

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Take a noisy reading

  measurementTaken = false;
  for lIdx = 1:length(real_landmarks)
    real_landmark = real_landmarks(:, lIdx);

    [z_real, G] = getMeasurement(real_position, real_landmark, measurement_variance);
    landmark_distance = z_real(1);
    landmark_angle    = z_real(2);

    % If the landmark is close enough, then we can spot it
    if(landmark_distance < max_read_distance)

      x = cos(landmark_angle)*landmark_distance + real_position(1);
      y = sin(landmark_angle)*landmark_distance + real_position(2);
      measurementTaken = true;
      for pIdx = 1:num_particles

        if(particles(pIdx).landmarks(lIdx).seen == false)
          particles(pIdx).landmarks(lIdx).pos = [particles(pIdx).position(1) + cos(landmark_angle)*landmark_distance;
                                                 particles(pIdx).position(2) + sin(landmark_angle)*landmark_distance;
                                                 0];
          particles(pIdx).landmarks(lIdx).E = inv(G) * R * inv(G)';
          particles(pIdx).landmarks(lIdx).seen = true;

        else
          % Get an ideal reading to our believed landmark position
          z_p = getMeasurement(particles(pIdx).position, particles(pIdx).landmarks(lIdx).pos, [0;0]);

          residual = z_real - z_p;

          x = cos(z_p(2))*landmark_distance + particles(pIdx).position(1);
          y = sin(z_p(2))*landmark_distance + particles(pIdx).position(2);

          Q = G' * particles(pIdx).landmarks(lIdx).E * G + R;

          %Calculate the Kalman gain
          K = particles(pIdx).landmarks(lIdx).E * G * Q;

          % Mix the ideal reading, and our actual reading using the Kalman gain, and use the result
          % to predict a new landmark position
          particles(pIdx).landmarks(lIdx).pos = particles(pIdx).landmarks(lIdx).pos + K*(residual); 

          % Update the covariance of this landmark
          particles(pIdx).landmarks(lIdx).E = (eye(size(K)) - K*G')*particles(pIdx).landmarks(lIdx).E;

          % Update the weight of the particle
          particles(pIdx).w = particles(pIdx).w * norm(2*pi*Q).^(-1/2)*exp(-1/2*(residual)'*inv(Q)*(residual));
        end
      end
    end
  end
  % Resample all particles based on their weights
  if(measurementTaken)
    %disp('Resampling');
    particles = resample(particles);
  end

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Plot our world
  clf;
  hold on;

  % Plot the landmarks
  for lIdx=1:length(real_landmarks)
    plot(real_landmarks(1,lIdx), real_landmarks(2,lIdx), 'b*');
  end

  for lIdx = 1:length(real_landmarks)
    if(particles(1).landmarks(lIdx).seen)
      avg_landmark_guess = particles(1).landmarks(lIdx).pos;
      min_w = inf;
      for pIdx = 2:num_particles
        if norm(particles(pIdx).w) < min_w
          avg_landmark_guess = avg_landmark_guess + particles(pIdx).landmarks(lIdx).pos;
        end
      end
      avg_landmark_guess = avg_landmark_guess / length(particles);
      plot(avg_landmark_guess(1), avg_landmark_guess(2), 'ko');
    end
  end

  % Plot the particles
  particles_pos = [particles.position];
  plot(particles_pos(1,:), particles_pos(2,:), 'r.');

  % Plot the real robot
  plot(pos_history(1,:), pos_history(2,:), 'r');
  plot(real_position(1), real_position(2), 'm*');
  quiver(real_position(1), real_position(2), ...
    cos(real_position(3))*arrow_length, ...
    sin(real_position(3))*arrow_length);

  % Show the sensor measurement as an arrow
  for lIdx=1:length(real_landmarks)
    real_landmark = real_landmarks(:, lIdx);
    [z_real, G] = getMeasurement(real_position, real_landmark, measurement_variance);
    landmark_distance = z_real(1);
    landmark_angle    = z_real(2);
    if(landmark_distance < max_read_distance)
      quiver(real_position(1), real_position(2), ...
            cos(landmark_angle)*landmark_distance, ...
            sin(landmark_angle)*landmark_distance);
    end
  end

  axis([-2, 2, -1, 4]);
  pause(.1);
end



