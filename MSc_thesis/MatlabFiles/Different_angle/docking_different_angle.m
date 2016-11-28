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

Pose_Matrix_data_1 = csvread('Pose_left.txt',1,0);
Vel_Matrix_data_1 = csvread('Velocity_left.txt',1,0);

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

Pose_Matrix_data_2 = csvread('Pose_center.txt',1,0);
Vel_Matrix_data_2 = csvread('Velocity_center.txt',1,0);


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

Pose_Matrix_data_3 = csvread('Pose_right.txt',1,0);
Vel_Matrix_data_3 = csvread('Velocity_right.txt',1,0);

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

% Reference Circle 
thresh_X = .001;
theta = 0:.001:2*pi;
% needs to be adjusted manually if docking platform is replaced!
ref_x = .201;
ref_y = .0138;
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
plot(t_P_sec_data_3,Pose_X_data_3,'g','LineWidth',1.6);

title('Pose estimation in marker coordinate system');
ylabel('${x_{mar}}$ [m]','interpreter','latex','FontSize',14);

l2 = legend('${Config. 3}$', '${Center}$','${Config. 4}$');
set(l2,'interpreter','latex','FontSize',11);
legend('boxoff');

subplot(3,1,2);
plot(t_P_sec_data_1,Pose_Y_data_1,'LineWidth',3);
hold on
plot(t_P_sec_data_2,Pose_Y_data_2,'r','LineWidth',1.7);
hold on
plot(t_P_sec_data_3,Pose_Y_data_3,'g','LineWidth',1.6);
ylabel('${y_{mar}}$ [m]','interpreter','latex','FontSize',14);

subplot(3,1,3);
plot(t_P_sec_data_1,Theta_data_1,'LineWidth',3);
hold on
plot(t_P_sec_data_2,Theta_data_2,'r','LineWidth',1.7);
hold on
plot(t_P_sec_data_3,Theta_data_3,'g','LineWidth',1.6);

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
plot(t_V_sec_data_3,Vel_X_data_3,'g','LineWidth',1.6);

title('Control Signals');
ylabel('$\dot{x}_{Rob}$ [m/s]','interpreter','latex','FontSize',14);
l2 = legend('${Config. 3}$', '${Center}$','${Config. 4}$');
set(l2,'interpreter','latex','FontSize',11);
legend('boxoff')

subplot(3,1,2);
plot(t_V_sec_data_1,Vel_Y_data_1,'LineWidth',3);
hold on
plot(t_V_sec_data_2,Vel_Y_data_2,'r','LineWidth',1.7);
hold on
plot(t_V_sec_data_3,Vel_Y_data_3,'g','LineWidth',1.6);
ylabel('$\dot{y}_{Rob}$ [m/s]','interpreter','latex','FontSize',14);


subplot(3,1,3);
plot(t_V_sec_data_1,Omega_Z_data_1,'LineWidth',3);
hold on
plot(t_V_sec_data_2,Omega_Z_data_2,'r','LineWidth',1.7);
hold on
plot(t_V_sec_data_3,Omega_Z_data_3,'g','LineWidth',1.6);
xlabel('$ time [sec]$','interpreter','latex','FontSize',14);
ylabel('$\dot{\theta}_{Rob}$ [rad/s]','interpreter','latex','FontSize',14);

% Trajectory 


% figure;
% plot(Pose_Y_data_1,Pose_X_data_1,'b',ref_Y_data_1,ref_X_data_1,'k*');
% hold on
% plot(Pose_Y_data_2,Pose_X_data_2,'r',ref_Y_data_2,ref_X_data_2,'k*');
% 
% hold on
% plot(Pose_Y_data_3,Pose_X_data_3,'g',ref_Y_data_3,ref_X_data_3,'k*');
% 

% 
% for k = Pose_X_data_1(1):Pose_X_data_1(size(Pose_X_data_1,1))
%     x_arrow(k) = 
%     
% end
% 
% x_arrow = linspace(Pose_X_data_1(1),Pose_X_data_1(size(Pose_X_data_1,1)),150);
% y_arrow = linspace(Pose_Y_data_1(1),Pose_Y_data_1(size(Pose_Y_data_1,1)),150);
% 
% Vx = gradient(x_arrow,.01);
% Vy = gradient(y_arrow,.01);
% 
% figure;
% plot(y_arrow,x_arrow,'b--','LineWidth',.5);
% % hold on
% % quiver(y_arrow,x_arrow,Vy,Vx,'r--','LineWidth',1.5);
% 
figure;
set(gcf,'color','white');

subplot(1,3,1);
plot(Pose_Y_data_1,Pose_X_data_1,'b','LineWidth',2);
% hold on
% quiver(Pose_Y_data_1,Pose_X_data_1,gradient(Pose_Y_data_1),gradient(Pose_X_data_1),'r--','LineWidth',.01);

hold on
plot(Pose_Y_data_2,Pose_X_data_2,'r','LineWidth',2);
% hold on
% quiver(Pose_Y_data_2,Pose_X_data_2,gradient(Pose_Y_data_2),gradient(Pose_X_data_2),'m--','LineWidth',.01);

hold on
plot(Pose_Y_data_3,Pose_X_data_3,'g','LineWidth',2);
% hold on
% quiver(Pose_Y_data_3,Pose_X_data_3,gradient(Pose_Y_data_3),gradient(Pose_X_data_3),'k--','LineWidth',.01);

hold on
plot(y_circle,x_circle,'k--','LineWidth',3.4);

title('Approach zone + SM zone + Target area');
ylabel('${x_{mar}}$ [m]','interpreter','latex','FontSize',14);
axis([-.4 .4 .18 1.4]);

subplot(1,3,2);
plot(Pose_Y_data_1,Pose_X_data_1,'b','LineWidth',1.6);
hold on
plot(Pose_Y_data_2,Pose_X_data_2,'r','LineWidth',1.6);
hold on
plot(Pose_Y_data_3,Pose_X_data_3,'g','LineWidth',1.6);
hold on
plot(y_circle,x_circle,'k--','LineWidth',3.4);

title('SM zone + Target area');
xlabel('${y_{mar}}$ [m]','interpreter','latex','FontSize',14);
axis([-.01 0.02 .195 .32]);

subplot(1,3,3);
plot(Pose_Y_data_1,Pose_X_data_1,'b','LineWidth',2);
hold on
plot(Pose_Y_data_2,Pose_X_data_2,'r','LineWidth',2);
hold on
plot(Pose_Y_data_3,Pose_X_data_3,'g','LineWidth',2);
hold on
plot(y_circle,x_circle,'k--','LineWidth',3.4);

title('Target area');
axis([0.0125 .015 .1995 .2055]);
l = legend('${Config. 3}$', '${Center}$','${Config. 4}$','Target');
set(l,'interpreter','latex','FontSize',11);



MaxOvershoot_1 = -ref_y + min(Pose_Y_data_1)
MaxOvershoot_2 = -ref_y + min(Pose_Y_data_2)
MaxOvershoot_3 = -ref_y + min(Pose_Y_data_3)
