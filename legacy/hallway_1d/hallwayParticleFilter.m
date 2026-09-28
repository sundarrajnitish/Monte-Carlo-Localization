function x = hallwayParticleFilter(x,modl,inpt)
% when using "motion model": set modl = 0, inpt = u
% when using "sensor model": set modl = 1, inpt = {0,1} for {no door, door}

if modl==0
  a3 = 0.01;
  %% Using normrnd
  x(:,1) = x(:,1) +inpt + normrnd(zeros(length(x(:,1)),1),sqrt(a3*abs(inpt)));
  
  % Using Irwin-Hall approximation
  %x(:,1) = x(:,1) + inpt + sum(sqrt(a3*abs(inpt))*(rand(length(x(:,1)),12)-0.5),2);
elseif modl==1
  % compare sensor reading to sensor model and assign weights
  if inpt==1
    x(:,2) = sensor_model(x(:,1));
  elseif inpt==0
    x(:,2) = 1-sensor_model(x(:,1));
  end
  %check if outside map
  x(x(:,1)>15,2)=0;
  
  % renormalize weights
  M = length(x(:,2));
  eta = M/sum(x(:,2));
  x(:,2) = eta*x(:,2);
  
  %resample particles
  %x = resample(x);
  x = LV_resample(x);
  
end
histogram(x(:,1),100);
axis([0 20 0 150]);