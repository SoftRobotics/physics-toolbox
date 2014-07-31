%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Desc		- This file defines the global parameters of simulation.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File		- globals.m
% Author	- Prashant Joshi ( joshi@igi.tu-graz.ac.at )
% Date		- 16-07-2003
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global DT TSTIM Xo Yo m1 lc1 I1 m2 l1 l2 lc2 I2 MFACT PERT_TIME DURATION FBDELAY

% The time-step of simulation.
DT = 0.001;

% The time of simulation
TSTIM = 0.5;


% The X coordinate of origin
Xo = 0;

% The Y coordinate of origin
Yo = 0;

% The mass of 1st link
m1 = 0;

% The mass of second link
m2 = 1;

% The distance of central point of link 1.
lc1 = 0.;

% The distance of central point of link 2.
lc2 = 0.;

% Length of first link
l1 = 10.;

% Length of second link
l2 = 10.;

% The moment of inertia of link1.
I1 = 0.03;

% The moment of inertia of link2.
I2 = 0.03;

% The factor by which the external noise will be 
% proportioanl to the input magnitude.
MFACT = 1e-2;

% The time at which the perturbation will start
PERT_TIME = 0;

% The duration of perturbation
DURATION = 0;

% The feedback delay
FBDELAY = 10;

% Number of input channels
NO_IP_CHANNELS = 300;

% No of training trajectories.
NO_TRAINING    = 400;

% NO of validation trajectories.
NO_VALIDATE    = 4;

% For setting temporal delay in IP channels.
%	0 -- No Temporal Delay, 
%	1 -- Temporally delayed IP.
IS_TEMPORAL_DELAY = 1;

% Should the torque IP be delayed.
IS_U_DELAY 			= 0;

% Define the file path for IP dist.
PATH = './inputs/@generalization_ip_class/private/';

% Array size to save on hard disk.
SAVE_BLOCK = 20;
