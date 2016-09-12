clc;
close all;
clear all;
%% Import data and time conversion

% data 1: 556 ros messages for geometry_msgs/Twist 
%         557 ros messages for geometry_msgs/PoseStamped
% y-axis gains:
% (010816_1718) best!!!
% P = .4; I = 0; D = 0; 

% y-axis gains:
% (280716_2024) better!!!
% P = .44; I = 0; D = 0; 

% theta-axis gains:
% P = .08; I = 0; D = 0;

% y-axis gains:
% (280716_2002)
% P = .46; I = 0; D = 0; 

% theta-axis gains:
% P = .08; I = 0; D = 0;

Pose_Matrix_data_1 = csvread('POSE_with_Py0.86_Iy0_Dy0.002ANDPt0.08_It0_Dt0.txt',1,0);
Vel_Matrix_data_1 = csvread('VEL_with_Py0.86_Iy0_Dy0.002ANDPt0.08_It0_Dt0.txt',1,0);

% time in position
TimeP_ros_data_1 = Pose_Matrix_data_1(:,1); % ros time, needs to be converted to sec...

% duriation =         end_time        -     start_time
  durationP_data_1 = (TimeP_ros_data_1(size(TimeP_ros_data_1,1),:) - TimeP_ros_data_1(1,:))*10^(-9); 
  
t_P_sec_data_1 = 0:durationP_data_1/size(TimeP_ros_data_1,1):durationP_data_1;
t_P_sec_data_1(:,size(t_P_sec_data_1,2)) = [];

% time in vel
TimeV_ros_data_1 = Vel_Matrix_data_1(:,1); % ros time, needs to be converted to sec...

% duriation =         end_time        -     start_time
  durationV_data_1 = (TimeV_ros_data_1(size(TimeV_ros_data_1,1),:) - TimeV_ros_data_1(1,:))*10^(-9);

t_V_sec_data_1 = 0:durationV_data_1/size(TimeV_ros_data_1,1):durationV_data_1;
t_V_sec_data_1(:,size(t_V_sec_data_1,2)) = [];

% -----------------------------------------------------------------------------------------------------
% data 2: 588 ros messages for geometry_msgs/Twist 
%         589 ros messages for geometry_msgs/PoseStamped
% y-axis gains:
% P = .5; I = 0.001; D = 0.03;

% theta-axis gains:
% P = .28; I = 0.1; D = 0.15;

Pose_Matrix_data_2 = csvread('POSE_with_Py0.66_Iy0_Dy0.1ANDPt0.08_It0_Dt0.txt',1,0);
Vel_Matrix_data_2 = csvread('VEL_with_Py0.66_Iy0_Dy0.1ANDPt0.08_It0_Dt0.txt',1,0);


% time in position
TimeP_ros_data_2 = Pose_Matrix_data_2(:,1); % ros time, needs to be converted to sec...

% duriation =         end_time        -     start_time
  durationP_data_2 = (TimeP_ros_data_2(size(TimeP_ros_data_2,1),:) - TimeP_ros_data_2(1,:))*10^(-9); 
  
t_P_sec_data_2 = 0:durationP_data_2/size(TimeP_ros_data_2,1):durationP_data_2;
t_P_sec_data_2(:,size(t_P_sec_data_2,2)) = [];

% time in vel
TimeV_ros_data_2 = Vel_Matrix_data_2(:,1); % ros time, needs to be converted to sec...

% duriation =         end_time        -     start_time
  durationV_data_2 = (TimeV_ros_data_2(size(TimeV_ros_data_2,1),:) - TimeV_ros_data_2(1,:))*10^(-9);

t_V_sec_data_2 = 0:durationV_data_2/size(TimeV_ros_data_2,1):durationV_data_2;
t_V_sec_data_2(:,size(t_V_sec_data_2,2)) = [];

% -----------------------------------------------------------------------------------------------------
% data 3: 538 ros messages for geometry_msgs/Twist 
%         539 ros messages for geometry_msgs/PoseStamped
% y-axis gains:
% P = .44; I = .0005; D = 0.15;

% theta-axis gains:
% P = .08; I = 0; D = 0;

Pose_Matrix_data_3 = csvread('POSE_with_Py0.86_Iy0_Dy0.1ANDPt0.08_It0_Dt0.txt',1,0);
Vel_Matrix_data_3 = csvread('VEL_with_Py0.86_Iy0_Dy0.1ANDPt0.08_It0_Dt0.txt',1,0);

% time in position
TimeP_ros_data_3 = Pose_Matrix_data_3(:,1); % ros time, needs to be converted to sec...

% duriation =         end_time        -     start_time
  durationP_data_3 = (TimeP_ros_data_3(size(TimeP_ros_data_3,1),:) - TimeP_ros_data_3(1,:))*10^(-9); 
  
t_P_sec_data_3 = 0:durationP_data_3/size(TimeP_ros_data_3,1):durationP_data_3;
t_P_sec_data_3(:,size(t_P_sec_data_3,2)) = [];

% time in vel
TimeV_ros_data_3 = Vel_Matrix_data_3(:,1); % ros time, needs to be converted to sec...

% duriation =         end_time        -     start_time
  durationV_data_3 = (TimeV_ros_data_3(size(TimeV_ros_data_3,1),:) - TimeV_ros_data_3(1,:))*10^(-9);

t_V_sec_data_3 = 0:durationV_data_3/size(TimeV_ros_data_3,1):durationV_data_3;
t_V_sec_data_3(:,size(t_V_sec_data_3,2)) = [];

% -----------------------------------------------------------------------------------------------------
% data 4: 450 ros messages for geometry_msgs/Twist 
%         451 ros messages for geometry_msgs/PoseStamped
% y-axis gains:
% P = .86; I = 0; D = 0.002;

% theta-axis gains:
% P = .08; I = 0; D = 0;

Pose_Matrix_data_4 = csvread('POSE_with_Py0.51_Iy0.0005_Dy0.05ANDPt0.08_It0_Dt0.txt',1,0);
Vel_Matrix_data_4 = csvread('VEL_with_Py0.51_Iy0.0005_Dy0.05ANDPt0.08_It0_Dt0.txt',1,0);

% time in position
TimeP_ros_data_4 = Pose_Matrix_data_4(:,1); % ros time, needs to be converted to sec...

% duriation =         end_time        -     start_time
  durationP_data_4 = (TimeP_ros_data_4(size(TimeP_ros_data_4,1),:) - TimeP_ros_data_4(1,:))*10^(-9); 
  
t_P_sec_data_4 = 0:durationP_data_4/size(TimeP_ros_data_4,1):durationP_data_4;
t_P_sec_data_4(:,size(t_P_sec_data_4,2)) = [];

% time in vel
TimeV_ros_data_4 = Vel_Matrix_data_4(:,1); % ros time, needs to be converted to sec...

% duriation =         end_time        -     start_time
  durationV_data_4 = (TimeV_ros_data_4(size(TimeV_ros_data_4,1),:) - TimeV_ros_data_4(1,:))*10^(-9);

t_V_sec_data_4 = 0:durationV_data_4/size(TimeV_ros_data_4,1):durationV_data_4;
t_V_sec_data_4(:,size(t_V_sec_data_4,2)) = [];

% -----------------------------------------------------------------------------------------------------
% data 5: 450 ros messages for geometry_msgs/Twist 
%         451 ros messages for geometry_msgs/PoseStamped
% y-axis gains:
% P = .86; I = 0; D = 0.1;

% theta-axis gains:
% P = .08; I = 0; D = 0;

Pose_Matrix_data_5 = csvread('Pose_with_Android.txt',1,0);
Vel_Matrix_data_5 = csvread('Velocity_with_Android.txt',1,0);

% time in position
TimeP_ros_data_5 = Pose_Matrix_data_5(:,1); % ros time, needs to be converted to sec...

% duriation =         end_time        -     start_time
  durationP_data_5 = (TimeP_ros_data_5(size(TimeP_ros_data_5,1),:) - TimeP_ros_data_5(1,:))*10^(-9); 
  
t_P_sec_data_5 = 0:durationP_data_5/size(TimeP_ros_data_5,1):durationP_data_5;
t_P_sec_data_5(:,size(t_P_sec_data_5,2)) = [];

% time in vel
TimeV_ros_data_5 = Vel_Matrix_data_5(:,1); % ros time, needs to be converted to sec...

% duriation =         end_time        -     start_time
  durationV_data_5 = (TimeV_ros_data_5(size(TimeV_ros_data_5,1),:) - TimeV_ros_data_5(1,:))*10^(-9);

t_V_sec_data_5 = 0:durationV_data_5/size(TimeV_ros_data_5,1):durationV_data_5;
t_V_sec_data_5(:,size(t_V_sec_data_5,2)) = [];


%% Pose estimation

%data 1:
% when using marker pose ,,,,
Pose_X_data_1 = Pose_Matrix_data_1(:,4);
Pose_Y_data_1 = Pose_Matrix_data_1(:,3);
Theta_data_1 = Pose_Matrix_data_1(:,5);


% when using robot odometry ,,,,
% Pose_X = Pose_Matrix(:,2);
% Pose_Y = Pose_Matrix(:,3);
% Theta = Pose_Matrix(:,7);

% Extracting reference values when the robot is manually docked!
ref_X_data_1 = Pose_X_data_1(size(Pose_X_data_1,1));
ref_Y_data_1 = Pose_Y_data_1(size(Pose_Y_data_1,1));
ref_Theta_data_1 = Theta_data_1(size(Theta_data_1,1));
ref_Pose_data_1 =[ref_X_data_1;ref_Y_data_1];

%data 2:
% when using marker pose ,,,,
Pose_X_data_2 = Pose_Matrix_data_2(:,4);
Pose_Y_data_2 = Pose_Matrix_data_2(:,3);
Theta_data_2 = Pose_Matrix_data_2(:,5);

% when using robot odometry ,,,,
% Pose_X = Pose_Matrix(:,2);
% Pose_Y = Pose_Matrix(:,3);
% Theta = Pose_Matrix(:,7);

% Extracting reference values when the robot is manually docked!
ref_X_data_2 = Pose_X_data_2(size(Pose_X_data_2,1));
ref_Y_data_2 = Pose_Y_data_2(size(Pose_Y_data_2,1));
ref_Theta_data_2 = Theta_data_2(size(Theta_data_2,1));
ref_Pose_data_2 =[ref_X_data_2;ref_Y_data_2];

%data 3:
% when using marker pose ,,,,
Pose_X_data_3 = Pose_Matrix_data_3(:,4);
Pose_Y_data_3 = Pose_Matrix_data_3(:,3);
Theta_data_3 = Pose_Matrix_data_3(:,5);

% when using robot odometry ,,,,
% Pose_X = Pose_Matrix(:,2);
% Pose_Y = Pose_Matrix(:,3);
% Theta = Pose_Matrix(:,7);

% Extracting reference values when the robot is manually docked!
ref_X_data_3 = Pose_X_data_3(size(Pose_X_data_3,1));
ref_Y_data_3 = Pose_Y_data_3(size(Pose_Y_data_3,1));
ref_Theta_data_3 = Theta_data_3(size(Theta_data_3,1));
ref_Pose_data_3 =[ref_X_data_3;ref_Y_data_3];

%data 4:
% when using marker pose ,,,,
Pose_X_data_4 = Pose_Matrix_data_4(:,4);
Pose_Y_data_4 = Pose_Matrix_data_4(:,3);
Theta_data_4 = Pose_Matrix_data_4(:,5);

% when using robot odometry ,,,,
% Pose_X = Pose_Matrix(:,2);
% Pose_Y = Pose_Matrix(:,3);
% Theta = Pose_Matrix(:,7);

% Extracting reference values when the robot is manually docked!
ref_X_data_4 = Pose_X_data_4(size(Pose_X_data_4,1));
ref_Y_data_4 = Pose_Y_data_4(size(Pose_Y_data_4,1));
ref_Theta_data_4 = Theta_data_4(size(Theta_data_4,1));
ref_Pose_data_4 =[ref_X_data_4;ref_Y_data_4];


%data 5:
% when using marker pose ,,,,
Pose_X_data_5 = Pose_Matrix_data_5(:,4);
Pose_Y_data_5 = Pose_Matrix_data_5(:,3);
Theta_data_5 = Pose_Matrix_data_5(:,5);

% when using robot odometry ,,,,
% Pose_X = Pose_Matrix(:,2);
% Pose_Y = Pose_Matrix(:,3);
% Theta = Pose_Matrix(:,7);

% Extracting reference values when the robot is manually docked!
ref_X_data_5 = Pose_X_data_5(size(Pose_X_data_5,1));
ref_Y_data_5 = Pose_Y_data_5(size(Pose_Y_data_5,1));
ref_Theta_data_5 = Theta_data_5(size(Theta_data_5,1));
ref_Pose_data_5 =[ref_X_data_5;ref_Y_data_5];



% Reference Circle 
thresh_X = .001;
theta = 0:.001:2*pi;
% needs to be adjusted manually if docking platform is replaced!
ref_x = .20095;
ref_y = .0081;
x_circle = thresh_X*cos(theta) + ref_x; % ref_X needs to be recorded
y_circle = thresh_X*sin(theta) + ref_y; % ref_Y needs to be recorded


%% Velocity estimation

% data 1:
Vel_X_data_1 = Vel_Matrix_data_1(:,2);
Vel_Y_data_1 = Vel_Matrix_data_1(:,3);
Omega_Z_data_1 = Vel_Matrix_data_1(:,7);

% data 2:
Vel_X_data_2 = Vel_Matrix_data_2(:,2);
Vel_Y_data_2 = Vel_Matrix_data_2(:,3);
Omega_Z_data_2 = Vel_Matrix_data_2(:,7);

% data 3:
Vel_X_data_3 = Vel_Matrix_data_3(:,2);
Vel_Y_data_3 = Vel_Matrix_data_3(:,3);
Omega_Z_data_3 = Vel_Matrix_data_3(:,7);

% data 4:
Vel_X_data_4 = Vel_Matrix_data_4(:,2);
Vel_Y_data_4 = Vel_Matrix_data_4(:,3);
Omega_Z_data_4 = Vel_Matrix_data_4(:,7);

% data 5:
Vel_X_data_5 = Vel_Matrix_data_5(:,2);
Vel_Y_data_5 = Vel_Matrix_data_5(:,3);
Omega_Z_data_5 = Vel_Matrix_data_5(:,7);

%% Plots

% for the presenation all pose estimation(x,y,theta) in one plot

% figure;
% plot(t_P_sec,Pose_X,'b',t_P_sec,Pose_Y,'r',t_P_sec,Theta,'g');
% 
% title('Marker Pose');
% xlabel('Time [sec]');
% ylabel('Pose');
% legend('X [m]','Y [m]','\theta [rad]');
% grid on

% marker position
figure;
set(gcf,'color','white');

subplot(3,1,1);
plot(t_P_sec_data_1,Pose_X_data_1,'LineWidth',3);
hold on
plot(t_P_sec_data_2,Pose_X_data_2,'r','LineWidth',1.7);
hold on
plot(t_P_sec_data_3,Pose_X_data_3,'g','LineWidth',2.3);
hold on
plot(t_P_sec_data_4,Pose_X_data_4,'m','LineWidth',1.6);
title('Pose estimation in marker coordinate system');
ylabel('${x_{mar}}$ [m]','interpreter','latex','FontSize',14);

l2 = legend('${P_y = 0.86, I_y = 0, D_y = 0.002}$', '${P_y = 0.66, I_y = 0, D_y = 0.1}$','${P_y = 0.86, I_y = 0, D_y = 0.1}$','${P_y = 0.51, I_y = 0.0005, D_y = 0.05}$');
set(l2,'interpreter','latex','FontSize',11);
legend('boxoff');

subplot(3,1,2);
plot(t_P_sec_data_1,Pose_Y_data_1,'LineWidth',3);
hold on
plot(t_P_sec_data_2,Pose_Y_data_2,'r','LineWidth',1.7);
hold on
plot(t_P_sec_data_3,Pose_Y_data_3,'g','LineWidth',2.3);
hold on
plot(t_P_sec_data_4,Pose_Y_data_4,'m','LineWidth',1.6);

ylabel('${y_{mar}}$ [m]','interpreter','latex','FontSize',14);

subplot(3,1,3);
plot(t_P_sec_data_1,Theta_data_1,'LineWidth',3);
hold on
plot(t_P_sec_data_2,Theta_data_2,'r','LineWidth',1.7);
hold on
plot(t_P_sec_data_3,Theta_data_3,'g','LineWidth',2.3);
hold on
plot(t_P_sec_data_4,Theta_data_4,'m','LineWidth',1.6);

xlabel('$ time [sec]$','interpreter','latex','FontSize',14);
ylabel('${\theta_{mar}}$ [rad]','interpreter','latex','FontSize',14);

% control signals
figure;
set(gcf,'color','white');

subplot(3,1,1);

plot(t_V_sec_data_1,Vel_X_data_1,'LineWidth',3);
hold on
plot(t_V_sec_data_2,Vel_X_data_2,'r','LineWidth',1.7);
hold on
plot(t_V_sec_data_3,Vel_X_data_3,'g','LineWidth',2.3);
hold on
plot(t_V_sec_data_4,Vel_X_data_4,'m','LineWidth',1.6);

title('Control Signals');
ylabel('$\dot{x}_{Rob}$ [m/s]','interpreter','latex','FontSize',14);
axis([0 45 -.01 .2]);

l2 = legend('${P_y = 0.86, I_y = 0, D_y = 0.002}$', '${P_y = 0.66, I_y = 0, D_y = 0.1}$','${P_y = 0.86, I_y = 0, D_y = 0.1}$','${P_y = 0.51, I_y = 0.0005, D_y = 0.05}$');
set(l2,'interpreter','latex','FontSize',11);
legend('boxoff');


subplot(3,1,2);
plot(t_V_sec_data_1,Vel_Y_data_1,'LineWidth',3);
hold on
plot(t_V_sec_data_2,Vel_Y_data_2,'r','LineWidth',1.7);
hold on
plot(t_V_sec_data_3,Vel_Y_data_3,'g','LineWidth',2.3);
hold on
plot(t_V_sec_data_4,Vel_Y_data_4,'m','LineWidth',1.6);
ylabel('$\dot{y}_{Rob}$ [m/s]','interpreter','latex','FontSize',14);

subplot(3,1,3);
plot(t_V_sec_data_1,Omega_Z_data_1,'LineWidth',3);
hold on
plot(t_V_sec_data_2,Omega_Z_data_2,'r','LineWidth',1.7);
hold on
plot(t_V_sec_data_3,Omega_Z_data_3,'g','LineWidth',2.3);
hold on
plot(t_V_sec_data_4,Omega_Z_data_4,'m','LineWidth',1.6);
xlabel('$ time [sec]$','interpreter','latex','FontSize',14);
ylabel('$\dot{\theta}_{Rob}$ [rad/s]','interpreter','latex','FontSize',14);


% Trajectory 

% 
% figure;
% plot(Pose_Y_data_1,Pose_X_data_1,'b',ref_Y_data_1,ref_X_data_1,'k*');
% hold on
% plot(Pose_Y_data_2,Pose_X_data_2,'r',ref_Y_data_2,ref_X_data_2,'k*');
% 
% hold on
% plot(Pose_Y_data_3,Pose_X_data_3,'g',ref_Y_data_3,ref_X_data_3,'k*');


% 
% for k = Pose_X_data_1(1):Pose_X_data_1(size(Pose_X_data_1,1))
%     x_arrow(k) = 
%     
% end
% 
% x_arrow = linspace(Pose_X_data_1(1),Pose_X_data_1(size(Pose_X_data_1,1)),150);
% y_arrow = linspace(Pose_Y_data_1(1),Pose_Y_data_1(size(Pose_Y_data_1,1)),150);
% 
% Vx = gradient(Pose_X_data_1);
% Vy = gradient(Pose_Y_data_1);

% figure;
% plot(Pose_Y_data_1,Pose_X_data_1,'b','LineWidth',3.5);
% hold on
% quiver(Pose_Y_data_1,Pose_X_data_1,Vy,Vx,'r--','LineWidth',.1);

figure;
set(gcf,'color','white');

subplot(1,3,1);
plot(Pose_Y_data_1,Pose_X_data_1,'b','LineWidth',2);
hold on
quiver(Pose_Y_data_1,Pose_X_data_1,gradient(Pose_Y_data_1),gradient(Pose_X_data_1),'r--','LineWidth',.1);

hold on
plot(Pose_Y_data_2,Pose_X_data_2,'r','LineWidth',2);
hold on
quiver(Pose_Y_data_2,Pose_X_data_2,gradient(Pose_Y_data_2),gradient(Pose_X_data_2),'m--','LineWidth',.1);

hold on
plot(Pose_Y_data_3,Pose_X_data_3,'g','LineWidth',2);
hold on
quiver(Pose_Y_data_3,Pose_X_data_3,gradient(Pose_Y_data_3),gradient(Pose_X_data_3),'k--','LineWidth',.1);

hold on
plot(Pose_Y_data_4,Pose_X_data_4,'m','LineWidth',2);
hold on
quiver(Pose_Y_data_4,Pose_X_data_4,gradient(Pose_Y_data_4),gradient(Pose_X_data_4),'b--','LineWidth',.1);

hold on
plot(y_circle,x_circle,'k--','LineWidth',3.4);

title('Approach zone + SM zone + Target area');
ylabel('${x}_{mar}$ [m]','interpreter','latex','FontSize',14);
axis([-.1 .4 .18 1.4]);

subplot(1,3,2);
plot(Pose_Y_data_1,Pose_X_data_1,'b','LineWidth',1.6);
hold on
plot(Pose_Y_data_2,Pose_X_data_2,'r','LineWidth',1.6);
hold on
plot(Pose_Y_data_3,Pose_X_data_3,'g','LineWidth',1.6);
hold on
plot(Pose_Y_data_4,Pose_X_data_4,'m','LineWidth',1.6);
hold on
plot(y_circle,x_circle,'k--','LineWidth',3.4);

title('SM zone + Target area');
xlabel('${y}_{mar}$ [m]','interpreter','latex','FontSize',14);
axis([-.005 0.045 .195 .32]);

subplot(1,3,3);
plot(Pose_Y_data_1,Pose_X_data_1,'b','LineWidth',2);
hold on
plot(Pose_Y_data_2,Pose_X_data_2,'r','LineWidth',2);
hold on
plot(Pose_Y_data_3,Pose_X_data_3,'g','LineWidth',2);
hold on
plot(Pose_Y_data_4,Pose_X_data_4,'m','LineWidth',2);
hold on
plot(y_circle,x_circle,'k--','LineWidth',3.4);

title('Target area');
axis([0.0065 .0095 .1995 .2055]);

l = legend('${P_y = 0.86, I_y = 0, D_y = 0.002}$', '${P_y = 0.66, I_y = 0, D_y = 0.1}$','${P_y = 0.86, I_y = 0, D_y = 0.1}$','${P_y = 0.51, I_y = 0.0005, D_y = 0.05}$' ,'Target');
set(l,'interpreter','latex','FontSize',11);
% legend('boxoff');


% sampling time
figure;
set(gcf,'color','white');

subplot(2,2,1);
plot(diff(TimeP_ros_data_3)/1e9,'r','LineWidth',1.4)
hold on
plot (diff(t_P_sec_data_3),'k--','LineWidth',1.8);
% hold on
title({'USB Camera';'Position'});
ylabel('Sampling time [sec]','interpreter','latex','FontSize',11);
axis([0 450 .068 .11]);
l3 = legend('${Time_{ROS}}$','${Time_{Real}} [sec]$','Orientation','horizontal');
set(l3,'interpreter','latex','FontSize',13);

subplot(2,2,2);
plot(diff(TimeP_ros_data_5)/1e9,'g','LineWidth',1.4)
hold on;
plot (diff(t_P_sec_data_5),'b--','LineWidth',1.8);

title({'Android Camera';'Position'});
axis([0 234 .068 .3]);

l4 = legend('${Time_{ROS}}$','${Time_{Real}} [sec]$','Orientation','horizontal');
set(l4,'interpreter','latex','FontSize',13);
legend('boxoff');

subplot(2,2,3);
plot(diff(TimeV_ros_data_3)/1e9,'r','LineWidth',1.4);
hold on;
plot (diff(t_V_sec_data_3),'k--','LineWidth',1.8);
% hold on;
title('Velocity');
xlabel('samples','interpreter','latex','FontSize',13);
ylabel('Sampling time [sec]','interpreter','latex','FontSize',13);
axis([0 450 .068 .11]);

subplot(2,2,4);
plot(diff(TimeV_ros_data_5)/1e9,'g','LineWidth',1.4);
hold on;
plot (diff(t_V_sec_data_5),'b--','LineWidth',1.8);

title('Velocity');
xlabel('samples','interpreter','latex','FontSize',13);
% ylabel('Sampling time [sec]','interpreter','latex','FontSize',11);
axis([0 234 .068 .3]);


figure;
set(gcf,'color','white');

subplot(2,1,1);
pa1 = plot(diff(TimeP_ros_data_3)/1e9,'r','LineWidth',2.1);
hold on
pa2 = plot (diff(t_P_sec_data_3),'k--','LineWidth',1.5);
hold on
pa3 = plot(diff(TimeP_ros_data_5)/1e9,'g','LineWidth',1.4);
hold on;
pa4 = plot (diff(t_P_sec_data_5),'b--','LineWidth',1.8);
hold off;
axis([0 234 0 .3]);
title('Position');
% ylabel('Sampling time [sec]','interpreter','latex','FontSize',13);
% 
% l5 = legend([pa1 pa2],'$USB Camera {Time_{ROS}}$','$ USB Camera {Time_{Real}} [sec]$','Orientation','horizontal');
% set(l5,'interpreter','latex','FontSize',11);

% leg = 
legend('ROS Time, USB camera','Real Time, USB camera [sec]','ROS Time, IP camera','Real Time, IP camera [sec]','Location','northwest');
legend('boxoff');

subplot(2,1,2);
p1 = plot(diff(TimeV_ros_data_3)/1e9,'r','LineWidth',2.1);
hold on;
p2 = plot (diff(t_V_sec_data_3),'k--','LineWidth',1.5);
hold on;
p3 = plot(diff(TimeV_ros_data_5)/1e9,'g','LineWidth',1.4);
hold on;
p4 = plot (diff(t_V_sec_data_5),'b--','LineWidth',1.8);
hold off;
axis([0 234 0 .3]);
title('Velocity');
% ylabel('Sampling time [sec]','interpreter','latex','FontSize',13);
% xlabel('Sample','interpreter','latex','FontSize',13);
% 
% l6 = legend([p3 p4],'$ Android Camera {Time_{ROS}}$','$ Android Camera {Time_{Real}} [sec]$','Orientation','horizontal');
% set(l6,'interpreter','latex','FontSize',12);

MaxOvershoot_1 = -ref_y + min(Pose_Y_data_1)
MaxOvershoot_2 = -ref_y + min(Pose_Y_data_2)
MaxOvershoot_3 = -ref_y + min(Pose_Y_data_3)
MaxOvershoot_4 = -ref_y + min(Pose_Y_data_4)
% Pose diff 
% figure;
% subplot(3,1,1);
% plot(diff(Pose_X_data_1));
% title('X-Pose diff');
% xlabel('samples');
% ylabel('X difference [m]');
% grid on
% 
% subplot(3,1,2);
% plot(diff(Pose_X_data_1));
% title('Y-Pose diff');
% xlabel('samples');
% ylabel('Y difference [m]');
% grid on
% 
% subplot(3,1,3);
% plot(diff(Theta_data_1));
% title('\theta-Pose diff');
% xlabel('samples');
% ylabel('\theta difference [rad]');
% grid on