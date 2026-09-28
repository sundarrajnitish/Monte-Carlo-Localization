function x_temp = LV_resample(x)

M = length(x(:,2));
x_temp = ones(M,2);
r = rand;
for ind = 1:M 
  jnd = 1;
  sumw = x(1,2);
  while sumw < r
    jnd = jnd+1;
    sumw = sumw + x(jnd,2);
  end
  x_temp(ind,1) = x(jnd,1); 
  r = r+1;  
end