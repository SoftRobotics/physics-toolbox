function [ns_t1, ns_t2, ns_w1, ns_w2] = torq2traj(t1, t2,...
												  w1, w2, u1, u2)

% input: 	u1,u2 ....torques
%  			w1,w2 ....angular velocities
%			t1,t2.....angles
% output:	ns_* .....next step values
%
% GLOBAL values needed for: 
%			m1,m2......masses
%			l1,l2......link lengths
%			lc1,lc2....center of mass position in local reference frame
%			I1,I2......inertia matrices
%			DT.........discrete time step
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Desc	- This file computes the next step of the robot arm trajectory
%			using the direct dynamics model. The equation for the direct
%			dynamics model is:
%			ALPHA = Inv(H).(TAU - C.OMEGA)
%			Where:
%			ALPHA is the Angular Acceleration Vector
%			H is the inertia matrix.
%			TAU is the torque vector.
%			C is the Centripetal and Coriolis force matrix.
%			OMEGA is the Angular Velocity Vector.
%
%			REFERENCE: Modelling and Control of Robotic Manipulators.
%						L. Sciavicco and B.Siciliano. Springer Publc.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File		- torq2traj.m
% Author	- Prashant Joshi ( joshi@igi.tu-graz.ac.at )
% Date		- 06/08/2003
% Adapted by helmut.hauser@igi.tugraz.at
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate the Inertia Matrix.
global m1 m2 l1 l2 lc1 lc2 I1 I2 DT

%  % Calculate the Inertia Matrix.
%  global ROBOTARM TIME_STEP
%  
%  m1 = ROBOTARM.m1;
%  m2 = ROBOTARM.m2;
%  l1 = ROBOTARM.l1;
%  l2 = ROBOTARM.l2;
%  lc1= ROBOTARM.lc1; 
%  lc2= ROBOTARM.lc2;
%  I1 = ROBOTARM.I1;
%  I2 = ROBOTARM.I2;
%  DT = TIME_STEP;
%  %  globals;
 
 
% Define the Inertia Matrix elements.
H11 = m1*lc1*lc1 + I1 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(t2)) + I2;
H12 = m2*l1*lc2*cos(t2) + m2*lc2*lc2 + I2;
H21 = H12;
H22 = m2*lc2*lc2 + I2;
 
% Define h
h = m2*l1*lc2*sin(t2);

H = [H11 H12;...
	 H21 H22];

% Compute its inverse.
IH = inv(H);

% Make the torque vector.
T = [u1 u2]';

% Compute the Centripetal and Coriolis force matrix.
C = [-h*w2	-h*(w1+w2);...
      h*w1  0];

% Make the Omega vector.
OMEGA = [w1 w2]';

% Now compute the ALPHA
ALPHA = IH*(T - C*OMEGA);

% Now start calculating the next step traj elems.
ns_w1 = ALPHA(1)*DT + w1;
ns_w2 = ALPHA(2)*DT + w2;

ns_t1 = w1*DT + t1;
ns_t2 = w2*DT + t2;
