% -------------------------------------------------------------------------
%                  ASSEMBLY LINE ROBOT - INVERSE KINEMATICS
% -------------------------------------------------------------------------
%   AUTHOR: Ajmal
%   DATE CREATED: 01/04/2022
%   DESCRIPTION: Takes an end-effector coordinate point and link parameters
%                and calculates the joint angles for each motor.
%                The robot modelled is a RRRR robot, consiting of 1 motor
%                with a verticle axis of rotation and 3 motors with
%                a horizontal axis of rotation.
% -------------------------------------------------------------------------

% Desired end-effector co-ord
x = 1;
y = 1;
z = 1;
yaw = 90;   % Direction to face

% Link parameters
l1 = 185; % First Link
l2 = 110; % Second Link
l3 = 115; % Third Link
l4 = 55; % Forth Link

% Conversion constant
radToDeg = 180/pi;

% Theta 1 calculations
cos_theta1 = x/sqrt(x^2 + y^2);
sin_theta1 = y/sqrt(x^2 + y^2);
theta1 = atan2(sin_theta1, cos_theta1);

% End-effector joint position calculations
x_prime_diff = cos(yaw)*l4;
z_diff = sin(yaw)*l4;
y_diff = sin(theta1)*x_prime_diff;
x_diff = cos(theta1)*x_prime_diff;
p_x = x - x_diff;
p_y = y - y_diff;
p_z = z - z_diff;

% Theta 3 calculations
x_prime = sqrt(p_x^2 + p_y^2);
z_prime = p_z - l1;
cos_theta3 = (x_prime^2 + z_prime^2 - l2^2 - l3^2)/(2*l2*l3);
sin_theta3 = real(sqrt(1 - cos_theta3^2));
theta3 = atan2(sin_theta3, cos_theta3);

% Theta 2 calculations
sin_beta = z_prime/sqrt(x_prime^2 + z_prime^2);
cos_beta = x_prime/sqrt(x_prime^2 + z_prime^2);
beta = atan2(sin_beta, cos_beta);
cos_phi = (x_prime^2 + z_prime^2 + l2^2 - l3^2)/(2*l2*sqrt(x_prime^2 + z_prime^2));
sin_phi = (l3/sqrt(x_prime^2 + z_prime^2))*sin_theta3;
phi = atan2(sin_phi, cos_phi);
theta2 = (beta - phi);

%theta 4 Calculations
theta4= 45/pi;
 
O_1x=0; 
O_1y=0; 
O_1z=0; 
O_2x=0; 
O_2y=0; 
O_2z=l1; 
O_3x=O_2x+l2*cos(theta2)*cos(theta1);
O_3y=O_2y+l2*cos(theta2)*sin(theta1); 
O_3z=O_2z+l2*sin(theta2); 
O_4x=O_3x+l3*cos(theta3)*cos(theta2);
O_4y=O_3y+l3*cos(theta3)*cos(theta2);
O_4z=O_3z+l3*sin(theta3);
p_x=25;
p_y=25;
p_z=25;


%plot the robot
XX=[O_1x O_2x O_3x O_4x p_x ];
YY=[O_1y O_2y O_3y O_4y p_y ];
ZZ=[O_1z O_2z O_3z O_4z p_z ];
XXj=[O_1x O_2x O_3x O_4x p_x ];
YYj=[O_1y O_2y O_3y O_4y p_y ];
ZZj=[O_1z O_2z O_3z O_4z p_z ];
plot3(XXj,YYj,ZZj,'o','LineWidth',2,'MarkerSize',10);
hold on;
plot3(XX,YY,ZZ, 'g','LineWidth',2);
hold off;