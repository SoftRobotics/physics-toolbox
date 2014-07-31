function [F] = make_simulation_movie(q,SAVE_MOVIE,name)  % with name and so on
%
% input: 	q			trajectory of the joint angles 2 x N
%			SAVE_MOVIE	boolean: 
%
%  helmut.hauser@igi.tugraz.at
%  2009-07-02 13:40:06

% defining default values
if (nargin==2)
	name = 'ms_movie';
end
if (nargin==1)
	name = 'ms_movie';
	SAVE_MOVIE = 0;
end

close all;
fps= 20;
time_step = 0.01;  % change the value to make it faster or slower!!!!!!!!
num_step = 1/fps/time_step; % 25 fps = 40 ms
F = moviein(size(q,2)/num_step);

% initial posture
show_posture(q(:,1),0);
lim = axis;

% video making loop
frame_nr = 0;
time_counter = 0;
    for idx=1:num_step:size(q,2)
    		idx
    		frame_nr=frame_nr+1; 
    		time_counter = time_counter+num_step*time_step;
    	    
			% plot all spring connections
			clf;
			hold on;
          	show_posture(q(:,idx),0);

          	hold off;
		 	axis(lim);
		 	title(['time = ' ,num2str(time_counter),' s'])
		  	drawnow;
%  			F(frame_nr) = getframe(gcf); % the whole figure (including title)
			F(frame_nr) = getframe;		 % just the plot
    end

% saving the file to out.avi 
% and make small version out of it
if (SAVE_MOVIE==1)	
	disp(['... saving file to ',name,'.avi']) 
 	movie2avi(F,[name,'_BIG.avi'],'fps',fps);
 	disp('... using mencoder to reduce filesize');
	unix(['mencoder -quiet ', name,'_BIG.avi -ovc lavc -o ',name,'.avi']);
	disp('... cleaning old big avi-file')
	unix(['rm -f ',name,'_BIG.avi']); 
	disp('finished.');
end

