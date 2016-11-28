clc;
close all;
clear all;

upper_angle = 135; % deg.
lower_angle = -135; % deg.
dist_to_obst = .1; % m.

%% docked robot for 5 different experiences...

S_mat = csvread('Scan_Data_5_exp.csv');

indx = S_mat(:,1);

s_exp_1 = S_mat(:,2);
s_exp_2 = S_mat(:,3);
s_exp_3 = S_mat(:,4);
s_exp_4 = S_mat(:,5);
s_exp_5 = S_mat(:,6);


points = size(indx,1); % to calculate the angle...
laser_beam = points -1;

angle = zeros(points,1);
X_exp_1 = zeros(points,1);
X_exp_2 = zeros(points,1);
X_exp_3 = zeros(points,1);
X_exp_4 = zeros(points,1);
X_exp_5 = zeros(points,1);
X_obst = zeros(points,1);

Y_exp_1 = zeros(points,1);
Y_exp_2 = zeros(points,1);
Y_exp_3 = zeros(points,1);
Y_exp_4 = zeros(points,1);
Y_exp_5 = zeros(points,1);
Y_obst = zeros(points,1);



for i = 1:points
    
    angle(i) = (lower_angle + (indx(i)* ((upper_angle - lower_angle)/(laser_beam))))* pi/180;
    
    X_exp_1(i) = s_exp_1(i) * cos(angle(i));
    X_exp_2(i) = s_exp_2(i) * cos(angle(i));
    X_exp_3(i) = s_exp_3(i) * cos(angle(i));
    X_exp_4(i) = s_exp_4(i) * cos(angle(i));
    X_exp_5(i) = s_exp_5(i) * cos(angle(i));
    X_obst(i) = dist_to_obst * cos(angle(i));
    
    Y_exp_1(i) = s_exp_1(i) * sin(angle(i));
    Y_exp_2(i) = s_exp_2(i) * sin(angle(i));
    Y_exp_3(i) = s_exp_3(i) * sin(angle(i));
    Y_exp_4(i) = s_exp_4(i) * sin(angle(i));
    Y_exp_5(i) = s_exp_5(i) * sin(angle(i));
    Y_obst(i) = dist_to_obst * sin(angle(i));    
end

% theta vs. scanner_value & reference
figure;
subplot(1,2,1);
% plot(angle,s_exp_1,'m--','LineWidth',1.1);
% hold on;
plot(angle,s_exp_2,'m--','LineWidth',1.9);
hold on;
plot(angle,s_exp_3,'b--','LineWidth',1.9);
hold on;
plot(angle,s_exp_4,'g--','LineWidth',1.1);
hold on;
% plot(angle,s_exp_5,'b--','LineWidth',.8);
% hold on;
plot(angle,dist_to_obst,'rx','LineWidth',.05);
title('');
xlabel('${Angle}$ [rad]','interpreter','latex','FontSize',14);
ylabel('${Distance [m]}$','interpreter','latex','FontSize',14);
axis([-.09 .42 .08 .2]);
% legend('Experiment 1','Experiment 2','Experiment 3','Safety Curve');
subplot(1,2,2);
% plot(X_exp_1,Y_exp_1,'m--','LineWidth',.9);
% hold on
plot(X_exp_2,Y_exp_2,'m--','LineWidth',1.9);
hold on
plot(X_exp_3,Y_exp_3,'b--','LineWidth',1.9);
hold on
plot(X_exp_4,Y_exp_4,'g--','LineWidth',1.1);
hold on
% plot(X_exp_5,Y_exp_5,'b--','LineWidth',.1);
% hold on
plot(X_obst,Y_obst,'r--','LineWidth',.05);
title('');
xlabel('${X}$ [m]','interpreter','latex','FontSize',14);
ylabel('${Y}$ [m]','interpreter','latex','FontSize',14);
axis([-.1 .16 -.18 .18]);
l = legend('Experiment 1','Experiment 2','Experiment 3','Safety Curve','Orientation','horizontal');
set(l,'interpreter','latex','FontSize',13);

%% docked robot with marker vs. docked robot without marker!
S_Mat_DockedwithMar = csvread('scanner_data_Docked_WITH_Marker.txt',0,0);
S_Mat_DockedwithNOmar = csvread('scanner_data_no_cylinder.txt',0,0);

indx_DockedwithMar = S_Mat_DockedwithMar(:,1);
s_val_DockedwithMar = S_Mat_DockedwithMar(:,2);

indx_DockedwithNOmar = S_Mat_DockedwithNOmar(:,1);
s_val_DockedwithNOmar = S_Mat_DockedwithNOmar(:,2);

points_DockedwithMar = size(indx_DockedwithMar,1);
points_DockedwithNOmar = size(indx_DockedwithNOmar,1);

laser_beam_DockedwithMar = points_DockedwithMar - 1;
laser_beam_DockedwithNOmar = points_DockedwithNOmar - 1;

s_angle_DockedwithMar = zeros(points_DockedwithMar,1);
s_angle_DockedwithNOmar = zeros(points_DockedwithNOmar,1);

X_DockedwithMar = zeros(points_DockedwithMar,1); Y_DockedwithMar = zeros(points_DockedwithMar,1);
X_DockedwithNOmar = zeros(points_DockedwithNOmar,1); Y_DockedwithNOmar = zeros(points_DockedwithNOmar,1);

X_obst_DockedwithMar = zeros(points_DockedwithMar,1); Y_obst_DockedwithMar = zeros(points_DockedwithMar,1);
X_obst_DockedwithNOmar = zeros(points_DockedwithNOmar,1); Y_obst_DockedwithNOmar = zeros(points_DockedwithNOmar,1);


for i = 1:points_DockedwithMar
    s_angle_DockedwithMar(i) = (lower_angle + (indx_DockedwithMar(i)* ((upper_angle - lower_angle)/(laser_beam_DockedwithMar))))* pi/180;
    X_DockedwithMar(i) = s_val_DockedwithMar(i) * cos(s_angle_DockedwithMar(i));
    Y_DockedwithMar(i) = s_val_DockedwithMar(i) * sin(s_angle_DockedwithMar(i));
    X_obst_DockedwithMar(i) = dist_to_obst * cos(s_angle_DockedwithMar(i));
    Y_obst_DockedwithMar(i)= dist_to_obst * sin(s_angle_DockedwithMar(i));
end


for i = 1:points_DockedwithNOmar
    s_angle_DockedwithNOmar(i) = (lower_angle + (indx_DockedwithNOmar(i)* ((upper_angle - lower_angle)/(laser_beam_DockedwithNOmar))))* pi/180;
    X_DockedwithNOmar(i) = s_val_DockedwithNOmar(i) * cos(s_angle_DockedwithNOmar(i));
    Y_DockedwithNOmar(i) = s_val_DockedwithNOmar(i) * sin(s_angle_DockedwithNOmar(i));
    X_obst_DockedwithNOmar(i) = dist_to_obst * cos(s_angle_DockedwithNOmar(i));
    Y_obst_DockedwithNOmar(i)= dist_to_obst * sin(s_angle_DockedwithNOmar(i));
end

% checking accuracy in robot coordinate system
figure;
subplot(3,1,1);
plot(X_DockedwithNOmar,Y_DockedwithNOmar);
hold on;
plot(X_obst_DockedwithNOmar,Y_obst_DockedwithNOmar,'r','LineWidth',3);
% xlabel('$ X [m] $','interpreter','latex','FontSize',14);
% ylabel('$ Y [m] $','interpreter','latex','FontSize',14);
title('No marker in docking platform');
legend('Obstacles','Laser scanner safety curve (r = 10 cm)');
% grid on
axis([-2.5 4 -4 1]);

subplot(3,1,2);
plot(X_DockedwithMar,Y_DockedwithMar,'b');
hold on;
plot(X_obst_DockedwithMar,Y_obst_DockedwithMar,'r','LineWidth',3);
ylabel('$ Y [m] $','interpreter','latex','FontSize',14);
title('Marker “circular bottle” in docking platform');
% grid on
axis([-2.5 4 -4 1]);

subplot(3,1,3);
plot(X_DockedwithMar,Y_DockedwithMar,'b');
hold on;
plot(X_obst_DockedwithMar,Y_obst_DockedwithMar,'r','LineWidth',.3);
xlabel('$ X [m] $','interpreter','latex','FontSize',14);
% ylabel('$ Y [m] $','interpreter','latex','FontSize',14);
% grid on
title('Detected marker');
axis([.04 .16 -.12 .12]);

% 
figure;
subplot(3,1,1);
plot(s_angle_DockedwithNOmar,s_val_DockedwithNOmar);
hold on
plot(s_angle_DockedwithNOmar,dist_to_obst,'r','LineWidth',.2);
% xlabel('Angle [rad]');
% ylabel('Distance to Obstacle [m]');
title('No marker in docking platform');

l2 = legend('${Obstacles}$', '${Safety Margin (r = 10 cm)}$');
set(l2,'interpreter','latex','FontSize',12);

axis([-2.2 2.2 -.5 5]);


subplot(3,1,2);
plot(s_angle_DockedwithMar,s_val_DockedwithMar);
hold on
plot(s_angle_DockedwithMar,dist_to_obst,'r','LineWidth',.2);
% xlabel('Angle [rad]');
ylabel('Distance to objects [m]');
title('Marker “circular bottle” in docking platform');
axis([-2.2 2.2 -.5 5]);

subplot(3,1,3);
plot(s_angle_DockedwithMar,s_val_DockedwithMar);
hold on
plot(s_angle_DockedwithMar,dist_to_obst,'r','LineWidth',10);
title('Detected marker');
xlabel('Angle [rad]');
% ylabel('Distance to Obstacle [m]');
axis([-.15 .35 .08 .15]);

%% The robot is moving Towards docking...!
Scanner_Matrix_Moving = csvread('scanner_data_Moving_Towards_Docking_WITH_Marker.txt',0,0);

index_Moving = Scanner_Matrix_Moving(:,1);
scanner_value_Moving = Scanner_Matrix_Moving(:,2);

index_Moving_Final = Scanner_Matrix_Moving(size(Scanner_Matrix_Moving,1),1);
scanner_value_Moving_Final = Scanner_Matrix_Moving(size(Scanner_Matrix_Moving,1),2);

% figure;
% plot(index_Moving,scanner_value_Moving,'m',index_Moving_Final,scanner_value_Moving_Final,'k*');
% legend('Distance to obstacle','Reference Position');
% grid on;
% xlabel('Index');
% ylabel('Distance to Obstacle [m]');
% title('Scanner Analysis for a moving robot!');
% axis([0 540 0 31]);


%% The robot is in the docking area, while NOT docked!
S_Mat_Out = csvread('scanner_data_outside.txt',0,0);
indx_out = S_Mat_Out(:,1);
s_val_out = S_Mat_Out(:,2);

points_out = size(indx_out,1);
laser_beam_out = points_out - 1;
s_angle_out = zeros(points_out,1);
X_out = zeros(points_out,1); Y_out = zeros(points_out,1);
X_obst_out = zeros(points_out,1); Y_obst_out = zeros(points_out,1);

for i = 1:points_out
    s_angle_out(i) = (lower_angle + (indx_out(i)* ((upper_angle - lower_angle)/(laser_beam_out))))* pi/180;
    X_out(i) = s_val_out(i) * cos(s_angle_out(i));
    Y_out(i) = s_val_out(i) * sin(s_angle_out(i));
    
    X_obst_out(i) = dist_to_obst * cos(s_angle_out(i));
    Y_obst_out(i)= dist_to_obst * sin(s_angle_out(i));
end

figure;
subplot(1,2,1);
plot(s_angle_out,s_val_out,'b','LineWidth',2);
hold on
plot(s_angle_out,dist_to_obst,'r','LineWidth',10);
xlabel('$Angle [rad]$','interpreter','latex','FontSize',14);
ylabel('$Obstacle [m]$','interpreter','latex','FontSize',14);
grid on;
axis([-2.23 2.23 -1 10]);

subplot(1,2,2)
plot(X_out,Y_out,'b','LineWidth',2);
hold on
plot(X_obst_out,Y_obst_out,'r-','LineWidth',4)
xlabel('$ X [m] $','interpreter','latex','FontSize',14);
ylabel('$ Y [m] $','interpreter','latex','FontSize',14);

l2 = legend('${Obstacles}$', '${Safety Margin (r = 10 cm)}$');
set(l2,'interpreter','latex','FontSize',12);

grid on;
axis([-1.2 5 -3.5 1.5]);


%% The robot is totally docked and a box is besides of laser scanner!
S_Mat_DockedwithBOX = csvread('scanner_data_box.txt',0,0);
indx_DockedwithBOX = S_Mat_DockedwithBOX(:,1);
s_val_DockedwithBOX = S_Mat_DockedwithBOX(:,2);

points_DockedwithBOX = size(indx_DockedwithBOX,1);
laser_beam_DockedwithBOX = points_DockedwithBOX - 1;
s_angle_DockedwithBOX = zeros(points_DockedwithBOX,1);

X_DockedwithBOX = zeros(points_DockedwithBOX,1); Y_DockedwithBOX = zeros(points_DockedwithBOX,1);
X_obst_DockedwithBOX = zeros(points_DockedwithBOX,1); Y_obst_DockedwithBOX = zeros(points_DockedwithBOX,1);

for i = 1:points_DockedwithBOX
    s_angle_DockedwithBOX(i) = (lower_angle + (indx_DockedwithBOX(i)* ((upper_angle - lower_angle)/(laser_beam_DockedwithBOX))))* pi/180;
    X_DockedwithBOX(i) = s_val_DockedwithBOX(i) * cos(s_angle_DockedwithBOX(i));
    Y_DockedwithBOX(i) = s_val_DockedwithBOX(i) * sin(s_angle_DockedwithBOX(i));
    
    
    X_obst_DockedwithBOX(i) = dist_to_obst * cos(s_angle_DockedwithBOX(i));
    Y_obst_DockedwithBOX(i)= dist_to_obst * sin(s_angle_DockedwithBOX(i));
end


figure;

subplot(2,1,1);
plot(s_angle_DockedwithBOX,s_val_DockedwithBOX,'b','LineWidth',.01);
hold on
plot(s_angle_DockedwithBOX,dist_to_obst, 'r','LineWidth',5);
% xlabel('Angle [rad]');
% ylabel('Distance to Obstacle [m]');
title('Marker, e.g., Box in docking platform');
legend('Surrounding objects','Laser scanner safety line (r = 10 cm)');
axis([-2.2 2.2 -.5 5]);

subplot(2,1,2);
plot(s_angle_DockedwithBOX,s_val_DockedwithBOX,'b','LineWidth',.01);
hold on
plot(s_angle_DockedwithBOX,dist_to_obst, 'r','LineWidth',5);
% xlabel('Angle [rad]');
% ylabel('Distance to Obstacle [m]');
axis([-1.9 0 .08 .22]);
title('Detected marker');


% 
% figure;
% plot(X_DockedwithBOX,Y_DockedwithBOX);