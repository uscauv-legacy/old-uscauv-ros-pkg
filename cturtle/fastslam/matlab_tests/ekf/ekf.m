timesteps = 120;
arrow_length = .5;

real_landmark = [1;     % x
                 2];    % y

real_position = [0;     % x
                 0;     % y
                 pi/2]; % rotation

movement_command = [.04;     % Distance
                    .00];    % Rotation
                    
movement_variance = [.01;   % Distance
                     .01]; % Rotation

observation_variance = [.05;   % Distance
                        .001]; % Angle

pos = real_position;

E = [0, 0, 0;
     0, 0, 0;
     0, 0, 0];

pos_history = [];

for timestep = 1:timesteps

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Move the actual robot
  [real_position, G, V] = ...
      moveParticle(real_position, movement_command, movement_variance);

  pos = moveParticle(pos, movement_command, [0;0]);

   M = [movement_variance(1), 0;
        0, movement_variance(2)];


   E = G*E*G' + V*M*V';


  pos_history = [pos_history, real_position];
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Take a reading
  [z, H] = ...
      getMeasurement(real_position, real_landmark, observation_variance);

  z_hat = ...
      getMeasurement(pos, real_landmark, [0; 0]);

  Q = [observation_variance(1), 0, 0;
       0, observation_variance(2), 0;
       0, 0, .0001];
        
  S = H*E*H' + Q;
  K = E*H'/ S;
   
  pos = pos + K*(z - z_hat);

  E = (eye(size(E)) - K*H)*E;


  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Plot our world
  clf;

  % Plot the landmark
  plot(real_landmark(1,:), real_landmark(2,:), 'b*');
  hold on;

  plot(pos(1), pos(2), 'r*');
  ellipse(E(1,1), E(2,2), 0, pos(1), pos(2), [1,0,0], 20);

  % Plot the real robot
  plot(pos_history(1,:), pos_history(2,:), 'r');
  plot(real_position(1), real_position(2), 'm*');
  quiver(real_position(1), real_position(2), ...
    cos(real_position(3))*arrow_length, ...
    sin(real_position(3))*arrow_length);

  % Show the sensor measurement as an arrow
  %if(landmark_distance < max_read_distance)
    quiver(real_position(1), real_position(2), ...
          cos(z(2)+real_position(3))*z(1), ...
          sin(z(2)+real_position(3))*z(1));
  %end

  axis([-2, 2, -1, 4]);
  pause(.1);
end



