timesteps = 200;
max_read_distance = 1.5;

real_landmarks = [1.0,  2.0,  0.0, 0.0, 1.0;     % x
                  3.0,  2.5   3.4, 1.5, 3.5;     % y
                  0.0,  0.0   0.0, 0.0, 0.0];    % Nuthin

real_position = [0.0;     % x
                 -1.0;     % y
                 pi/3.0]; % rotation

movement_command = [.05;     % Distance
                    .00];    % Rotation
                    
movement_variance = [.1;   % Distance
                     .5]; % Rotation

M = [movement_variance(1), 0.0;
     0.0, movement_variance(2)];

measurement_variance = [0.1; % Distance
                        0.01; % Angle
                        .0001]; % Landmark Identity

R = [measurement_variance(1), 0.0, 0.0;
     0.0, measurement_variance(2), 0.0;
     0.0, 0.0, measurement_variance(3)];

% Create the particles 
particles = [];
num_particles = 100;
for i = 1:num_particles
  particles(i).w = 1.0/num_particles;
  particles(i).position = real_position;
  for lIdx=1:size(real_landmarks,2)
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
     particles(pIdx).position(3) = real_position(3);
  end

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Take a noisy reading

  doResample = false;
  for lIdx = 1:size(real_landmarks,2)
    real_landmark = real_landmarks(:, lIdx);

    %disp('perfect:');
    z_perfect = getMeasurement(real_position, real_landmark, [0 0]);
    real_distance(lIdx) = z_perfect(1);
    real_angle(lIdx)    = z_perfect(2);

    %disp('real:');
    [z_real, G] = getMeasurement(real_position, real_landmark, measurement_variance);
    read_distance(lIdx) = z_real(1);
    read_angle(lIdx)    = z_real(2);


    % If the landmark is close enough, then we can spot it
    if(real_distance(lIdx) < max_read_distance)
      doResample = true;

      perf_x = cos(z_perfect(2))*z_perfect(1) + real_position(1);
      perf_y = sin(z_perfect(2))*z_perfect(1) + real_position(2);
      read_x = cos(z_real(2))*z_real(1) + real_position(1);
      read_y = sin(z_real(2))*z_real(1) + real_position(2);
      %disp(['Landmark: (',num2str(perf_x),',',num2str(perf_y),') ~ (',num2str(read_x),',',num2str(read_y),')']);

      x = cos(read_angle(lIdx))*read_distance(lIdx) + real_position(1);
      y = sin(read_angle(lIdx))*read_distance(lIdx) + real_position(2);
      for pIdx = 1:num_particles

        if(particles(pIdx).landmarks(lIdx).seen == false)
          % If we have never seen this landmark, then we need to initialize it
          particles(pIdx).landmarks(lIdx).pos = [particles(pIdx).position(1) + cos(read_angle(lIdx))*read_distance(lIdx);
                                                 particles(pIdx).position(2) + sin(read_angle(lIdx))*read_distance(lIdx);
                                                 0];
          particles(pIdx).landmarks(lIdx).E = inv(G) * R * inv(G)';
          particles(pIdx).landmarks(lIdx).seen = true;

          partlan_x = particles(pIdx).landmarks(lIdx).pos(1);
          partlan_y = particles(pIdx).landmarks(lIdx).pos(2);
          %disp([' InitPos: (',num2str(partlan_x),',',num2str(partlan_y),')']);

        else
          % Get an ideal reading to our believed landmark position
          %disp('particle:');
          [z_p, Gp] = getMeasurement(particles(pIdx).position, particles(pIdx).landmarks(lIdx).pos, [0;0]);
          residual = z_real - z_p;

          %Calculate the Kalman gain
          Q = G' * particles(pIdx).landmarks(lIdx).E * G + R;
          K = particles(pIdx).landmarks(lIdx).E * G * inv(Q);

          % Mix the ideal reading, and our actual reading using the Kalman gain, and use the result
          % to predict a new landmark position
          particles(pIdx).landmarks(lIdx).pos = particles(pIdx).landmarks(lIdx).pos + K*(residual); 

          % Update the covariance of this landmark
          particles(pIdx).landmarks(lIdx).E = (eye(size(K)) - K*G')*particles(pIdx).landmarks(lIdx).E;

          % Update the weight of the particle
          particles(pIdx).w = particles(pIdx).w * norm(2*pi*Q).^(-1/2)*exp(-1/2*(residual)'*inv(Q)*(residual));
        end %else
      end %pIdx
    end %distance


  end %for landmark
  % Resample all particles based on their weights
  if(doResample)
    particles = resample(particles);
  end

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Plot our world
  clf;
  hold on;

  % Plot the landmarks
  for lIdx=1:size(real_landmarks,2)
    plot(real_landmarks(1,lIdx), real_landmarks(2,lIdx), 'b*');
  end

  for lIdx = 1:size(real_landmarks,2)
    if(particles(1).landmarks(lIdx).seen)
      avg_landmark_guess =[0;0;0];
      for pIdx = 1:length(particles)
        avg_landmark_guess = avg_landmark_guess + particles(pIdx).landmarks(lIdx).pos;
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
  w = .1;
  l = .3;
  x = real_position(1);
  y = real_position(2);
  t = real_position(3);
  plot(real_position(1), real_position(2), 'mo', ...
                                           'LineWidth',1.5, ...
                                           'MarkerEdgeColor','k', ...
                                           'MarkerFaceColor',[0 1 0], ...
                                           'MarkerSize',10);

  % Show the sensor measurement as an arrow
  for lIdx=1:size(real_landmarks,2)
    real_landmark = real_landmarks(:, lIdx);
    if(real_distance(lIdx) < max_read_distance)
      line([real_position(1), real_position(1)+cos(read_angle(lIdx))*read_distance(lIdx)], ...
           [real_position(2), real_position(2)+sin(read_angle(lIdx))*read_distance(lIdx)]);
    end
  end

  axis([-3, 3, -2, 5]);
  pause(.1);
end



