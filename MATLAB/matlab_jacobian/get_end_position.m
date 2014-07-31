function [p] = get_end_position(q)
%	calculates the the position p=[x,y]' of the
%	end effector of the two link robot arm
%	
% 2009-07-03 
% helmut.hauser@igi.tugraz.at
global l1 l2

p(1,:) = l1*cos(q(1,:))+l2*cos(q(1,:)+q(2,:));
p(2,:) = l1*sin(q(1,:))+l2*sin(q(1,:)+q(2,:));


