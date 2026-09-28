function x_temp = resample(x)

M = length(x(:,2));
x_temp = ones(M,2);
for ind = 1:M
  r = M*rand;
  jnd = 1;
  sumw = x(1,2);
  while sumw < r
    jnd = jnd+1;
    sumw = sumw + x(jnd,2);
  end
  x_temp(ind,1) = x(jnd,1);    
end