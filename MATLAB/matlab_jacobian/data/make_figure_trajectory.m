%  making the figure 8 trajectory 
%  in (x/y)
%  
%  2009-07-02
%  helmut.hauser@igi.tugraz.at

global DT

close all
time_step = DT;
total_time = 16;
t=linspace(0,total_time,total_time/time_step);

%  defining the offsets and amplitudes of the trajectory
x_off = 0.2;
x_amp = 0.6/2;  % between 0.2 and 0.8
y_off = 0.2;
y_amp = 0.4/2;  % between 0.2 and 0.6

% assuming one figure 8 in total_time
num_periods = 5;  % number of periods in the total time

x = x_off+x_amp+x_amp*sin(2*pi*1/total_time*num_periods*t);
y = y_off+y_amp+y_amp*sin(2*pi*2/total_time*num_periods*t);

figure;plot(t,x,'LineWidth',2);hold on;plot(t,y,'r','LineWidth',2);legend('x','y');xlabel('time [s]')
figure;plot(x,y,'LineWidth',2);xlabel('x');ylabel('y');title('limit cycle - figure 8')
text(0.35,0.6,['total time = ',num2str(total_time),' s'],'FontSize',22);
dat.x = x;
dat.y = y;
dat.time_step = time_step;
dat.total_time = total_time;
