function [dat_returned] = get_angles_from_xytraj(dat,q_init)
%
% input:	q_init		2x1  initial angles (starting point)
%						default = [1.5506 -1.7518] = [0.5/0.4] center of figure 8
%			dat			data structure with dat.x and dat.y with xy-trajectory
%
% output:	dat_returned   same data structure as in the input but now with found angles
%
% helmut.hauser@igi.tugraz.at
% 2009-07-02 

%  TODO: adapted to get real time video (timestep and fps!!)
if (nargin==1)
	disp('using default starting value= [1.5506 -1.7518] = [0.5/0.4](x/y)');
	q_init =[1.5506 -1.7518]';  % to start at 0.5 0.4 (center of figure 8)
end


% get xy trajectory data
x=dat.x(:)';
y=dat.y(:)';
p = vertcat(x,y); % and combine it newly

% init right values
q_found = q_init;
p_found = p;	

for j=1:length(x)-1
	w(:,j) = get_angle_direction(q_found(:,j),p_found(:,j),p(:,j+1));
	q_found(:,j+1) = q_found(:,j)+w(:,j);
    get_end_position(q_found(:,j+1));
	p_found(:,j+1) = get_end_position(q_found(:,j+1));

end

t = linspace(0,dat.total_time,dat.total_time/dat.time_step);
%  plot_trajectory
%figure;plot(p_found(1,:),p_found(2,:),'LineWidth',3);
%figure;plot(t,q_found(1,:),'LineWidth',3);
%hold on;plot(t,q_found(2,:),'r','LineWidth',3);

% add found q values to data structure
dat_returned   = dat;
dat_returned.q =  q_found;



