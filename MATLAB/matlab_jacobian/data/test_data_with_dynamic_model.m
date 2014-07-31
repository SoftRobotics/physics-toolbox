%  testing the found angle-traj-data with the dynamic model
%  by using a PD controller to get the right tau trajectories
addpath('../');


close all;
clear all;
robotarm_globals;
load('figure_trajectory.mat');
clear w;
%  w(1,:) = diff(q_found(1,:)); 
%  w(2,:) = diff(q_found(2,:)); 

% initial velocity
w1 = 0;
w2 = 0;
 
q_ = zeros(2,length(q_found));
q_(1,1) = q(1,1);
q_(2,1) = q(2,1);

% defining PD values - maybe have be changed 
% when using faster x/y trajectories
%  P=1; % works with 2ms
%  D=0.2; % works with 2ms
P=20;		% works fine with 1ms
D=0.8;		% works fine with 1ms
for j=1:length(q_found)-1
	e1 = q_(1,j)-q_found(1,j);e2 = q_(2,j)-q_found(2,j);
	tau(1,j) = -P*e1 - D*w1;
	tau(2,j) = -P*e2 - D*w2;
	[q_(1,j+1), q_(2,j+1), w1, w2] = torq2traj(q_(1,j),q_(2,j),w1, w2, tau(1,j), tau(2,j));
	p(:,j)  = get_end_position(q_(:,j+1));
end

figure;plot(p(1,:),p(2,:));
p_target = get_end_position(q_found);
hold on;plot(p_target(1,:),p_target(2,:),'r')
figure;plot(q_(1,:),q_(2,:));