M = 500;
x = [15*rand(M,1) ones(M,1)];
histogram(x(:,1),100);
axis([0 20 0 150]);
%sigma_door = 0.2;
%s = 0:0.01:15;
%pzs = 0.05+0.9*(exp(-(s-3).^2./2/(sigma_door)^2)+exp(-(s-5).^2./2/(sigma_door)^2)+exp(-(s-10).^2./2/(sigma_door)^2));
%figure,plot(s,pzs)

hallwayParticleFilter(x, 0, 10);