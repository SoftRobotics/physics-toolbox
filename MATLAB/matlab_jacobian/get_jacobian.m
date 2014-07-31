function [J] = get_jacobian(q)
%
% returns the Jacobian a q=[alpha,beta]
% of a two link robotarm
% reference: http://staff.science.uva.nl/~leo/math/jacobian.ps
% file in the same directory


global l1 l2 
%l1=0.5;l2=0.5;
q=q(:);
J = zeros(2,2);




J(1,1) = -l1*sin(q(1,1))  - l2*sin(q(1,1)+q(2,1));
J(1,2) = - l2*sin(q(1,1)+q(2,1));
J(2,1) = l1*cos(q(1,1))   + l2*cos(q(1,1)+q(2,1));
J(2,2) = l2*cos(q(1,1)+q(2,1));
