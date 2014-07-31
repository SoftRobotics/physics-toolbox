l1 = 10; % length of first arm
l2 = 10; % length of second arm

theta1 = 0:0.1:2*pi; % all possible theta1 values
theta2 = 0:0.1:2*pi; % all possible theta2 values

[THETA1, THETA2] = meshgrid(theta1, theta2); % generate a grid of theta1 and theta2 values

X = l1 * cos(THETA1) + l2 * cos(THETA1 + THETA2); % compute x coordinates
Y = l1 * sin(THETA1) + l2 * sin(THETA1 + THETA2); % compute y coordinates

data1 = [X(:) Y(:) THETA1(:)]; % create x-y-theta1 dataset
data2 = [X(:) Y(:) THETA2(:)]; % create x-y-theta2 dataset

plot(X(:), Y(:), 'r.');
  axis equal;
  xlabel('X','fontsize',10)
  ylabel('Y','fontsize',10)