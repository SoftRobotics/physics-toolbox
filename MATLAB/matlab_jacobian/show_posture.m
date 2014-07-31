

function [] = show_posture(q,MAKE_NEW_FIGURE)
%  	script to show posture of the robot arm 
%	with the corredponding values of the angles and end position coordinates
%
% input: q 					= [q1 q2] angles of joints in [rad]
%		 MAKE_NEW_FIGURE	boolean: 0 for moviemaking (default is 1)
%									in order open a new figure or not
%
%  2009-07-02 
%  helmut.hauser@igi.tugraz.at

% get global robotarm structure
global l1 l2 Xo Yo

% default is making a new figure
if (nargin==1)
 MAKE_NEW_FIGURE = 1;
end


q=q(:);
q1=q(1,1);
q2=q(2,1);

% calculating coordinatesW
x1 = Xo + cos(q1)*l1;
y1 = Yo + sin(q1)*l1;
x2 = Xo + cos(q1)*l1+cos(q1+q2)*l2;
y2 = Yo + sin(q1)*l1+sin(q1+q2)*l2;
if (MAKE_NEW_FIGURE)
	figure;
end	
plot([Xo x1 ],[Yo  y1],'b','LineWidth',10);
hold on;
plot([x1 x2],[y1 y2],'r','LineWidth',10);
plot(x2,y2,'ok','LineWidth',6)
%  axis equal;
axis([-(l1+l2) l1+l2 -(l1+l2) l1+l2]);
grid;
% to test get_end_position function
p = get_end_position(q);
hold on;plot(p(1),p(2),'+g','LineWidth',1)


h1=text(0,-0.5,['p =[ ',num2str(p(1)),' / ',num2str(p(2)), ']']);
h2=text(0,-0.7,['alpha = ',num2str(q(1))]);
h3=text(0,-0.8,['beta   = ',num2str(q(2))]);
FONTSIZE = 12;
set(h1,'FontSize',FONTSIZE,'FontWeight','Bold')
set(h2,'FontSize',FONTSIZE,'FontWeight','Bold')
set(h3,'FontSize',FONTSIZE,'FontWeight','Bold')