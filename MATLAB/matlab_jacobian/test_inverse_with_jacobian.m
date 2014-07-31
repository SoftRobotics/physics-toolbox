%  test inverse with Jacobian
% find your way along the line

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% this one works
q=[pi/2 -pi/2]';
p_start = [0.5 0.5]';
w  = get_angle_direction(q,p_start,[0.501 0.501]); % using K=1 
q=q+w;
get_end_position(q);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%  TODO: adapted to get real time video (timestep and fps!!)
%  testing with a loop
x=linspace(0.5,0.5,100);
y=linspace(0.5,0.4,100);
p = vertcat(x,y);
q=[pi/2 -pi/2]'; % to start 
q_found = q;
p_found = p;	

for j=1:length(x)-1
	w = get_angle_direction(q_found(:,j),p_found(:,j),p(:,j+1)); 
	q_found(:,j+1)=q_found(:,j)+w;
	p_found(:,j+1) = get_end_position(q_found(:,j+1));
end

% plot_trajectory
figure;plot(p_found(1,:),p_found(2,:),'LineWidth',3);
  figure;plot(q_found(1,:),'LineWidth',3);
  hold on;plot(q_found(2,:),'r','LineWidth',3);
