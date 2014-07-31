function [w] = get_angle_direction(q,p_start,p_end);
% calculate the angle velocities w=[w1 w2] (angle direction) 
% to get from p_start (with angels q) to p_end
%
% helmut.hauser@igi.tugraz.at
%  2009-07-02 11:53:46

K=1;
p_end = p_end(:);
p_start = p_start(:);

%  p = p_start;
%  while p
v = p_end - p_start; % velocity vector
J = get_jacobian(q);
w = inv(J)*K*(v);
