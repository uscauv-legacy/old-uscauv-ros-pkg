timesteps = 60;
arrow_length = .5;
max_read_distance = 1.5;

real_landmark = [1;     % x
                 2;
                 0];    % y

real_position = [0;     % x
                 0;     % y
                 pi/4]; % rotation

movement_command = [.07;     % Distance
                    .03];    % Rotation
                    
movement_variance = [.05;   % Distance
                     .035]; % Rotation

M = [movement_variance(1), 0;
     0, movement_variance(2)];

measurement_variance = [.0001; % Distance
                        .0001; % Angle
                        0]; % Nuthin

R = [measurement_variance(1), 0, 0;
     0, measurement_variance(2), 0;
     0, 0, measurement_variance(3)];


% Create the particles 
num_particles = 300;
for i = 1:num_particles

  % Set the particle position
  particles(i).position = real_position;

  % Set the particles believed landmark position
  particles(i).landmark_position = real_landmark + ...
    measurement_variance .* randn(size(measurement_variance));

  % Initialize the particle's landmark covariance
  particles(i).E = [measurement_variance(1), 0,0;
                    0, measurement_variance(2),0;
                    0, 0, measurement_variance(3)];
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
  [z_real, G] = getMeasurement(real_position, real_landmark, measurement_variance);
  landmark_distance = z_real(1);
  landmark_angle    = z_real(2);

  % If the landmark is close enough, then we can spot it
  if(landmark_distance < max_read_distance)
    for pIdx = 1:num_particles

      % Get an ideal reading to our believed landmark position
      z_p = getMeasurement(particles(pIdx).position, particles(pIdx).landmark_position, [0;0]);
      Q = G' * particles(pIdx).E  * G + R;
      K = particles(pIdx).E * G * Q;

      particles(pIdx).landmark_position = particles(pIdx).landmark_position + K*(z_real-z_p);
      particles(pIdx).E = (eye(size(K)) - K*G')*particles(pIdx).E;
      particles(pIdx).w = norm(2*pi*Q).^(-1/2)*exp(-1/2*(z_real-z_p)'*inv(Q)*(z_real-z_p));

    end
    particles = resample(particles);
  end

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Plot our world
  clf;

  % Plot the landmark
  plot(real_landmark(1,:), real_landmark(2,:), 'b*');
  hold on;

  min_cov = Inf;
  best_landmark_guess = [-1;-1];
  for pIdx = 1:num_particles
    if norm(particles(pIdx).E) < min_cov
      best_landmark_guess = particles(pIdx).landmark_position;
    end
  end
  plot(best_landmark_guess(1), best_landmark_guess(2), 'go');

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
  if(landmark_distance < max_read_distance)
    quiver(real_position(1), real_position(2), ...
          cos(landmark_angle+real_position(3))*landmark_distance, ...
          sin(landmark_angle+real_position(3))*landmark_distance);
  end

  axis([-2, 2, -1, 4]);
  pause(.1);
end



