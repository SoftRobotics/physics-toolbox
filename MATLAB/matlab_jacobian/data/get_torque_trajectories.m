function [tau,dat_returned] = get_torque_trajectories(dat)
%  caculate the approriate torques depending on a start position q and
%  a desired q testing now the the found data with the dynamic model
%  and a PD controller to get finally the right tau trajectories
%
% input:	q	2xN trajectories of the angles
%				default: load values form "figure8_angles.mat"
%
% output:	tau 			2xN tau trajectories which lead to q
%		 	dat_returned	data structure from input but now additional with torques
%
%				
% 2009-07-03
% helmut.hauser@igi.tugraz.at

if(nargin==0)
	disp('loading data ... figure8_xy_5times.mat');
	load('figure8_xy_5times.mat');	
	q = dat.q;
else
	q =dat.q;
end

addpath('../');
close all;

% initial velocity
w1 = 0;
w2 = 0;
 
% q_ finally found angles
q_ = zeros(size(q));
q_(1,1) = q(1,1); % init with starting values
q_(2,1) = q(2,1);

% defining PD values - maybe have be changed 
% when using faster x/y trajectories
%  P=1; % works with 2ms
%  D=0.2; % works with 2ms
%%%%%%%%%%%%%%%%%%%%%%%%
P=20;		% works fine with 1ms
D=0.8;		% works fine with 1ms
%%%%%%%%%%%%%%%%%%%%%%%%%%%
P=200;		% works OK with 16/5 periods
D=2;		% works fine with 1ms
for j=1:length(q)-1
	e1 = q_(1,j)-q(1,j);e2 = q_(2,j)-q(2,j);
	tau(1,j) = -P*e1 - D*w1;
	tau(2,j) = -P*e2 - D*w2;
	[q_(1,j+1), q_(2,j+1), w1, w2] = torq2traj(q_(1,j),q_(2,j),w1, w2, tau(1,j), tau(2,j));
	p(:,j)  = get_end_position(q_(:,j+1));
end
%  
figure;plot(p(1,:),p(2,:));
p_target = get_end_position(q);
hold on;plot(p_target(1,:),p_target(2,:),'r')

dat_returned = dat;
dat_returned.tau = tau;






