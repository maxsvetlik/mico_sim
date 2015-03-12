l1 = .275; % length from base to first joint
l2 = .290; % length from first joint to 2nd joint
l3 = .1233;
l4 = .0741; % length of j4 at 60 deg angle from j3
l5 = .0741; % length of j5 to j6, also at 60 deg angle
l6 = .160; % length of end effector

%theta limits
t1 = 0:0.1:2*pi; % all possible theta1 values
t2 = 0:0.1:pi; % all possible theta2 values
t3 = 0:0.1:.75*pi;
t4 = 0:0.1:2*pi;
t5 = 0:0.1:2*pi;
t6 = 0:0.1:2*pi;

%[THETA1, THETA2, THETA3, THETA4, THETA5, THETA6] = meshgrid(t1, t2, t3, t4, t5, t6); % generate a grid of theta1 and theta2 values
%t1 = 0;
%t2 = -90;
%t3 = 0;
%t4 = 0;
%t5 = 0;
%t6 = 0;
THETA_sample = [0,90,0,0,0,0];
syms t1 t2 t3 t4 t5 t6
%X = [0  + l2 .* cos(THETA1) .* cos(THETA2) + l3*sin(THETA3)]; 
%Y = [l1 + l1 .* sin(THETA2)                + l3*cos(THETA3)];
%Z = [0  + l1 .* sin(THETA1) .* cos(THETA2) + cos(THETA1)];

A1 = [ cos(t1), -cos(90)*sin(t1),  sin(90)*sin(t1),0;
 sin(t1),  cos(90)*cos(t1), -sin(90)*cos(t1),      0;
       0,          sin(90),          cos(90), 0.0755;
       0,                0,                0,      1];

A2 = [ cos(t2), -sin(t2), 0, 0.29*cos(t2);
     sin(t2),  cos(t2), 0, 0.29*sin(t2);
       0,        0, 1,            0;
       0,        0, 0,            1];
   
A3 = [cos(t3), -cos(90)*sin(t3),  sin(90)*sin(t3), 0.1233*cos(t3);
 sin(t3),  cos(90)*cos(t3), -sin(90)*cos(t3), 0.1233*sin(t3);
       0,          sin(90),          cos(90),              0;
       0,                0,                0,              1];
 
 
A4 = [ cos(t4), -cos(60)*sin(t4),  sin(60)*sin(t4), 0.0741*cos(t4);
 sin(t4),  cos(60)*cos(t4), -sin(60)*cos(t4), 0.0741*sin(t4);
       0,          sin(60),          cos(60),        0.03705;
       0,                0,                0,              1];
 
 
A5 =[ cos(t5), -cos(60)*sin(t5),  sin(60)*sin(t5), 0.0741*cos(t5);
 sin(t5),  cos(60)*cos(t5), -sin(60)*cos(t5), 0.0741*sin(t5);
       0,          sin(60),          cos(60),        0.03705;
       0,                0,                0,              1];
 
 
A6 =[ cos(t6), -sin(t6), 0,    0;
 sin(t6),  cos(t6), 0,    0;
       0,        0, 1, 0.16;
       0,        0, 0,    1];

%D-H translation matrix   
T = A1 * A2 * A3 * A4 * A5 * A6;

%cartesian position of EF
x_s = T(1,4);
y_s = T(2,4);
z_s = T(3,4);

%'target' EF position
x_g = 1.902;
y_g = 2.01;
z_g = 1.3;

%absolute difference in cart space
dx = ((x_g - x)^2 + (y_g - y)^2 + (z_g - z)^2)^(.5);

%load symbolic position equations - `prejacobian`
V_T = [x_s, y_s, z_s];

%find jacobian
J = jacobian(V_T, [t1, t2, t3, t4, t5, t6]);
J = subs(J,{t1, t2, t3, t4, t5, t6}, THETA_sample);
vpa(J, 5);

%Calculate the pseudo inverse of jacobian
J_trans = transpose(J);
J_PI = J_trans*inv((J*J_trans));

%calculate the error 
I = eye([3,3]);
%error = ABS((I - (J*J_PI))*dx);
%vpa(error, 4);
thetas = J_PI*dx;
thetas = subs(thetas,{t1, t2, t3, t4, t5, t6}, THETA_sample);
vpa(thetas, 4)

%To get new THETA, add new to old : new thetas in 'thetas' as last column
%reshape(THETA_sample,6,1) + thetas

%J=subs(J,{t1,t2,t3,t4,t5,t6},{ones(3,1)},0);
%simplify(J*inv(J))
%inv(J)


%Old graphing functionality
%data1 = [X(:) Y(:) Z(:) THETA1(:)]; % create x-y-theta1 dataset
%data2 = [X(:) Y(:) Z(:) THETA2(:)]; % create x-y-theta2 dataset
%data3 = [X(:) Y(:) Z(:) THETA3(:)];
%plot3(X(:), Z(:), Y(:), 'r.');
%  axis equal;
%  xlabel('X','fontsize',10)
%  ylabel('Z','fontsize',10)
%  zlabel('Y','fontsize',10)
%  title('X-Y-Z For A Robotic Manipulator','fontsize',10)