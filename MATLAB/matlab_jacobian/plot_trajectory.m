% plot the x,y-trajectory of the found angles

figure;hold on;
for j=1:length(q_found)
	p = get_end_position(q_found(:,j));
	plot(p(1,1),p(2,1),'.');
end