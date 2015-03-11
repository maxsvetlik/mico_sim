l1 = .275; % length from base to first joint
l2 = .290; % length from first joint to 2nd joint
l3 = .1233
l4 = .0741 % length of j4 at 60 deg angle from j3
l5 = .0741 % length of j5 to j6, also at 60 deg angle
l6 = .160 % length of end effector

theta1 = 0:0.1:2*pi; % all possible theta1 values
theta2 = 0:0.1:pi; % all possible theta2 values
theta3 = 0:0.1:.75*pi;

[THETA1, THETA2, THETA3] = meshgrid(theta1, theta2, theta3); % generate a grid of theta1 and theta2 values

%X = l1 * cos(THETA1) + l2 * cos(THETA1 + THETA2); % compute x coordinates
%Y = l1 * sin(THETA1) + l2 * sin(THETA1 + THETA2); % compute y coordinates
X = [0  + l2 .* cos(THETA1) .* cos(THETA2) + l3*sin(THETA3)]; 
Y = [l1 + l1 .* sin(THETA2)                + l3*cos(THETA3)];
Z = [0  + l1 .* sin(THETA1) .* cos(THETA2) + cos(THETA1)];


data1 = [X(:) Y(:) Z(:) THETA1(:)]; % create x-y-theta1 dataset
data2 = [X(:) Y(:) Z(:) THETA2(:)]; % create x-y-theta2 dataset
data3 = [X(:) Y(:) Z(:) THETA3(:)];
plot3(X(:), Z(:), Y(:), 'r.');
  axis equal;
  xlabel('X','fontsize',10)
  ylabel('Z','fontsize',10)
  zlabel('Y','fontsize',10)
  title('X-Y-Z For A Robotic Manipulator','fontsize',10)