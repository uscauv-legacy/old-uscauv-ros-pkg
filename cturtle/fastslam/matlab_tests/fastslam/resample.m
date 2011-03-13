function [newParticles] = resample(oldParticles)
  
  weightSum = 0;
  for i=1:length(oldParticles)
    weightSum = weightSum + oldParticles(i).w;
  end
  for i=1:length(oldParticles)
    oldParticles(i).w = oldParticles(i).w/weightSum;
  end

  M = length(oldParticles);

  newParticles = [];

  r = rand / M;

  c = oldParticles(1).w;

  i = 1;

  for m=1:M
    U = r + (m-1) * M^(-1);

    while U > c
      i = i+1;
      c = c+oldParticles(i).w;
    end
    newParticles = [newParticles, oldParticles(i)];
  end

  for i=1:length(newParticles)
    newParticles(i).w = 1.0/length(newParticles);
  end

end
