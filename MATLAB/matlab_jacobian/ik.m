global l1 l2
l1 = 10;
l2 = 10;
x = 15;
y = 5;
dt=0.001;%2*pi/2000;

t = 0:dt:2*pi;

%RESET
X = 0;
Y = 0;

%X = 8*cos(t) + x;
%Y = 7*sin(2*t) / 2 + y;
%X = 5*sin(2*t) / 2 + x;
%Y = 6*cos(t) + y;
X = 4*cos(t)+x;
Y = 4*sin(t)+y;

trajectory.x = X(:);
trajectory.y = Y(:);
trajectory.total_time = 2*pi;
trajectory.time_step = dt;

figure;plot(X(:), Y(:), 'r.');
axis equal;

x=X(1);
y=Y(1);

b = sqrt(x.^2 + y.^2);
alpha = 2*pi - acos( (b.^2 + l1^2 - l2^2) ./ (2*b*l1));
beta  = pi - acos((l1^2 + l2^2 - b.^2) ./ (2*l1*l2));

angles = get_angles_from_xytraj(trajectory, [(alpha + atan(y./x)) beta]');
angles = transpose(angles.q);
angles = [diff(angles(:,1)) diff(angles(:,2))];
%angles = [angles(:,1) angles(:,2)];




csvwrite('myfile.csv',angles);

%FK to test
X = l1 * cos(angles(:,1)) + l2 * cos((angles(:,1) + angles(:,2)));
Y = l1 * sin(angles(:,1)) + l2 * sin((angles(:,1) + angles(:,2)));

figure;plot(X(:), Y(:), 'r.');
axis equal;
