
close all;

G=tf(1,poly([0.9 0.9 0.9]),0.001);
[u,t]=impulse(G);
%  t=0:0.01:0.6;


%  plot(y);

T=zeros(2,length(u));
W=zeros(2,length(u));


%  [ns_t1, ns_t2, ns_w1, ns_w2] = torq2traj(t1, t2,...
%  												  w1, w2, u1, u2)


  robotarm_globals;
  disp('loading needed values');



for i=2:length(u);
	[T(1,i),T(2,i),W(1,i),W(2,i)] = torq2traj(T(1,i-1),T(2,i-1),W(1,i-1),W(2,i-1),u(i),u(i));
end

T=T';
plot(T);
size(T(:,1))
[x,y]=theta2xy(T(:,1),T(:,2));
figure;plot(x);hold on;plot(y,'r')
figure;plot(x,y);

