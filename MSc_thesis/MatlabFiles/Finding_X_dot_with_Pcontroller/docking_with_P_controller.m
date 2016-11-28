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

Pose_Matrix_data_1 = csvread('Pose01_08_16_1718.txt',1,0);
Vel_Matrix_data_1 = csvread('Velocity01_08_16_1718.txt',1,0);

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

Pose_Matrix_data_2 = csvread('Pose01_08_16_1749.txt',1,0);
Vel_Matrix_data_2 = csvread('Velocity01_08_16_1749.txt',1,0);

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

Pose_Matrix_data_3 = csvread('Pose01_08_16_1831.txt',1,0);
Vel_Matrix_data_3 = csvread('Velocity01_08_16_1831.txt',1,0);

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

thresh_X = .001;
theta = 0:.001:2*pi;

% needs to be adjusted manually if docking platform is replaced!
ref_x = .2;
ref_y = .0085;

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
subplot(3,1,1);
plot(t_P_sec_data_1,Pose_X_data_1);
hold on
plot(t_P_sec_data_2,Pose_X_data_2,'r');
hold on
plot(t_P_sec_data_3,Pose_X_data_3,'g');

title('Marker X-axis','Color','r');
% xlabel('Time [sec]');
ylabel('X [m]');
grid on

subplot(3,1,2);
plot(t_P_sec_data_1,Pose_Y_data_1);
hold on
plot(t_P_sec_data_2,Pose_Y_data_2,'r');
hold on
plot(t_P_sec_data_3,Pose_Y_data_3,'g');

title('Marker Y-axis');
% xlabel('Time [sec]');
ylabel('Y [m]');
grid on

subplot(3,1,3);
plot(t_P_sec_data_1,Theta_data_1);
hold on
plot(t_P_sec_data_2,Theta_data_2,'r');
hold on
plot(t_P_sec_data_3,Theta_data_3,'g');

title('Marker \theta-axis');
xlabel('Time [sec]');
ylabel('\theta [rad]');
grid on



% control signals
figure;
set(gcf,'color','white');

subplot(3,1,1);

plot(t_V_sec_data_1,Vel_X_data_1,'LineWidth',2);
hold on
plot(t_V_sec_data_2,Vel_X_data_2,'r','LineWidth',1.6);
hold on
plot(t_V_sec_data_3,Vel_X_data_3,'g','LineWidth',2);

title('Control Signals');
ylabel('$\dot{x}_{Rob}$ [m/s]','interpreter','latex','FontSize',14);
% grid on
axis([0 60 -.01 .2]);
l = legend('$\dot{x}_{Rob}$ = 0.15 [m/s]','$\dot{x}_{Rob}$ = 0.1[m/s]','$\dot{x}_{Rob}$ = 0.16[m/s]');
set(l,'interpreter','latex','FontSize',12);
legend('boxoff');

subplot(3,1,2);
plot(t_V_sec_data_1,Vel_Y_data_1,'LineWidth',2.5);
hold on
plot(t_V_sec_data_2,Vel_Y_data_2,'r','LineWidth',1.5);
hold on
plot(t_V_sec_data_3,Vel_Y_data_3,'g','LineWidth',2.1);

% title('Control Signal  y_{Rob} - axis');
ylabel('$\dot{y}_{Rob}$ [m/s]','interpreter','latex','FontSize',14);
% grid on
% axis([0 46 -.2 .1]);

subplot(3,1,3);
plot(t_V_sec_data_1,Omega_Z_data_1,'LineWidth',2.5);
hold on
plot(t_V_sec_data_2,Omega_Z_data_2,'r','LineWidth',1.45);
hold on
plot(t_V_sec_data_3,Omega_Z_data_3,'g','LineWidth',2.1);

% title('Control Signal  \theta_{Rob} - axis');
xlabel('$ time [sec]$','interpreter','latex','FontSize',14);
ylabel('$\dot{\theta}_{Rob}$ [rad/s]','interpreter','latex','FontSize',14);
% grid on

% axis([0 46 -.1 .15]);

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
hold on
plot(Pose_Y_data_2,Pose_X_data_2,'r','LineWidth',2);
hold on
plot(Pose_Y_data_3,Pose_X_data_3,'g','LineWidth',2);
hold on
plot(y_circle,x_circle,'k--','LineWidth',3.4)

title('Approach zone + SM zone + Target area');
xlabel('${y}_{mar}$ [m]','interpreter','latex','FontSize',14);
ylabel('${x}_{mar}$ [m]','interpreter','latex','FontSize',14);
axis([-.3 .4 .18 1.4]);
% grid on
l2 = legend('$\dot{x}_{Rob}$ = 0.15[m/s]','$\dot{x}_{Rob}$ = 0.1[m/s]','$\dot{x}_{Rob}$ = 0.16[m/s]','Target','Location','northwest');
set(l2,'interpreter','latex');
legend('boxoff');

subplot(1,3,2);
plot(Pose_Y_data_1,Pose_X_data_1,'b','LineWidth',2);
hold on
plot(Pose_Y_data_2,Pose_X_data_2,'r','LineWidth',2);
hold on
plot(Pose_Y_data_3,Pose_X_data_3,'g','LineWidth',2);
hold on
plot(y_circle,x_circle,'k--','LineWidth',3.4);
title('SM zone + Target area');
xlabel('${y}_{mar}$ [m]','interpreter','latex','FontSize',14);
% ylabel('${X}$ [m]','interpreter','latex');
axis([-.01 0.06 .195 .32]);
% grid on

subplot(1,3,3);
plot(Pose_Y_data_1,Pose_X_data_1,'b','LineWidth',2);
hold on
plot(Pose_Y_data_2,Pose_X_data_2,'r','LineWidth',2);
hold on
plot(Pose_Y_data_3,Pose_X_data_3,'g','LineWidth',2);
hold on
plot(y_circle,x_circle,'k--','LineWidth',3.4);
title('Target area');
xlabel('${y}_{mar}$ [m]','interpreter','latex','FontSize',14);
% ylabel('${X}$ [m]','interpreter','latex');
axis([0.006 .011 .198 .204]);
% grid on
% l2 = legend('x_dot = 0.15','x_dot = 0.1','x_dot = 0.16','Reference Position');

figure;
set(gcf,'color','white');

subplot(2,1,1);
plot(diff(TimeP_ros_data_1)/1e9,'g')
hold on
plot (diff(t_P_sec_data_1),'k--')
title('Position sampling time');
ylabel('Sampling time');
legend('ROS calc.','Real time calc. [sec]','Orientation','horizontal');
% grid on
% axis([0 428 .05 .15]);

subplot(2,1,2);
plot(diff(TimeV_ros_data_1)/1e9,'g');
hold on
plot (diff(t_V_sec_data_1),'k--')
title('Velocity sampling time');
xlabel('samples');
ylabel('Sampling time');
% legend('ROS calc.','Ordinary calc.','Orientation','horizontal');
% grid on
% axis([0 428 -.005 .025]);
% 
% subplot(3,1,3);
% plot(diff(TimeV_ros_data_1)/1e9,'g')
% hold on
% plot (diff(t_V_sec_data_1),'k--')
% xlabel('samples');
% ylabel('time step');
% axis([150 250 0 .02]);
figure;
set(gcf,'color','white');

subplot(3,1,1);
plot(diff(Pose_X_data_1));
title('X-Pose diff');
xlabel('samples');
ylabel('X difference [m]');
grid on

subplot(3,1,2);
plot(diff(Pose_X_data_1));
title('Y-Pose diff');
xlabel('samples');
ylabel('Y difference [m]');
grid on

subplot(3,1,3);
plot(diff(Theta_data_1));
title('\theta-Pose diff');
xlabel('samples');
ylabel('\theta difference [rad]');
grid on