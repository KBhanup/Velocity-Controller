%% Clear everything
clear; close all; clc;

bag_file = 'LogWhen_Flying_2022-05-16-17-22-08.bag';
rot_data = rosbag(bag_file);
rosbag info 'LogWhen_Flying_2022-05-16-17-22-08.bag'
%rosbag info 'Feb14.bag'

load('Discreet_Z_10_9.mat');
load('Continious_Z_10_9.mat');

% JOY Data
JOY_select = select(rot_data, 'Topic', '/joy');
JOY_struct = readMessages(JOY_select, 'DataFormat', 'struct');
JOY_data = table();
JOY_data.time = ...
    cellfun(@(m) double(m.Header.Stamp.Sec), JOY_struct) + ...
    cellfun(@(m) double(m.Header.Stamp.Nsec)*1e-9, JOY_struct);
% AXES
JOY_Axes_data.px= cellfun(@(m) double(m.Axes(5)), JOY_struct);
JOY_Axes_data.py= cellfun(@(m) double(m.Axes(4)), JOY_struct);
JOY_Axes_data.pz= cellfun(@(m) double(m.Axes(2)), JOY_struct);
JOY_Axes_data.qw= cellfun(@(m) double(m.Axes(1)), JOY_struct);
% Buttons
JOY_Buttons_data.arm= cellfun(@(m) double(m.Buttons(6)), JOY_struct);
JOY_Buttons_data.offb= cellfun(@(m) double(m.Buttons(5)), JOY_struct);



% Local position from local
loc_setp_select = select(rot_data, 'Topic', '/mavros/setpoint_raw/local');
loc_setp_struct = readMessages(loc_setp_select, 'DataFormat', 'struct');
setpoint = table();
setpoint.time = ...
    cellfun(@(m) double(m.Header.Stamp.Sec), loc_setp_struct) + ...
    cellfun(@(m) double(m.Header.Stamp.Nsec)*1e-9, loc_setp_struct);
start_time = setpoint.time(1);
setpoint.time = setpoint.time-setpoint.time(1);
setpoint.velx = cellfun(@(m) double(m.Velocity.X), loc_setp_struct);
setpoint.vely = cellfun(@(m) double(m.Velocity.Y), loc_setp_struct);
setpoint.velz = cellfun(@(m) double(m.Velocity.Z), loc_setp_struct);

% From setpoints time we can calc dt=t2-t1;
dt=0.0501;

% Local_Position_Odom/// Velocities
locodom_select = select(rot_data, 'Topic', '/mavros/local_position/odom');
locodom_struct = readMessages(locodom_select, 'DataFormat', 'struct');
locodom_data = table();
locodom_data.time = ...
    cellfun(@(m) double(m.Header.Stamp.Sec), locodom_struct) + ...
    cellfun(@(m) double(m.Header.Stamp.Nsec)*1e-9, locodom_struct);
locodom_data.time= locodom_data.time - start_time;
locodom_data.time= locodom_data.time - locodom_data.time(1);
locodom_data.x = cellfun(@(m) double(m.Twist.Twist.Linear.X), locodom_struct);
locodom_data.y = cellfun(@(m) double(m.Twist.Twist.Linear.Y), locodom_struct);
locodom_data.z = cellfun(@(m) double(m.Twist.Twist.Linear.Z), locodom_struct);

locodom_Posedata.x = cellfun(@(m) double(m.Pose.Pose.Position.X), locodom_struct);
locodom_Posedata.y = cellfun(@(m) double(m.Pose.Pose.Position.Y), locodom_struct);
locodom_Posedata.z = cellfun(@(m) double(m.Pose.Pose.Position.Z), locodom_struct);

locodom_Orindata.w = cellfun(@(m) double(m.Pose.Pose.Orientation.W), locodom_struct);
locodom_Orindata.x = cellfun(@(m) double(m.Pose.Pose.Orientation.X), locodom_struct);
locodom_Orindata.y = cellfun(@(m) double(m.Pose.Pose.Orientation.Y), locodom_struct);
locodom_Orindata.z = cellfun(@(m) double(m.Pose.Pose.Orientation.Z), locodom_struct);

%%%%%To calc velocities only those related to setpoints
Bdy.Velx = interp1(locodom_data.time, locodom_data.x, setpoint.time, 'spline');
Bdy.Vely = interp1(locodom_data.time, locodom_data.y, setpoint.time, 'spline');
Bdy.Velz = interp1(locodom_data.time, locodom_data.z, setpoint.time, 'spline');
%%%
Bdy_wrld.Posex = interp1(locodom_data.time, locodom_Posedata.x, setpoint.time, 'spline');
Bdy_wrld.Posey = interp1(locodom_data.time, locodom_Posedata.y, setpoint.time, 'spline');
Bdy_wrld.Posez = interp1(locodom_data.time, locodom_Posedata.z, setpoint.time, 'spline');
% 
Bdy_wrld.qw = interp1(locodom_data.time, locodom_Orindata.w, setpoint.time, 'spline');
Bdy_wrld.qx = interp1(locodom_data.time, locodom_Orindata.x, setpoint.time, 'spline');
Bdy_wrld.qy = interp1(locodom_data.time, locodom_Orindata.y, setpoint.time, 'spline');
Bdy_wrld.qz = interp1(locodom_data.time, locodom_Orindata.z, setpoint.time, 'spline');

wld_qw = Bdy_wrld.qw;
wld_qx = Bdy_wrld.qx;
wld_qy = Bdy_wrld.qy;
wld_qz = Bdy_wrld.qz;






outputx = [];
inputx = setpoint.velx;
outputx = Bdy.Velx;
% outputx = outputx';

outputy = [];
inputy = setpoint.vely;
outputy = Bdy.Vely;


outputz = [];
inputz = setpoint.velz;
outputz =Bdy.Velz;






% %Creating a Transfer Function Model
% 
% sys_d=idtf(Discreet_Z_10_9.Numerator, Discreet_Z_10_9.Denominator, 0.0501);
% sys_c= idtf(Continius_Z_10_9.Numerator, Continius_Z_10_9.Denominator, 0.0501);
% 
% 
% 
% %% PID USing Code
% 
% D = pidtune(sys_d, 'PI');
% C = pidtune(sys_c, 'P');


%%% State Space System only for x axis

% Define System Dynamics F,G
F =zeros([6,6]);
F(1,1) = 1;F(1,4) = 0.0501;
F(2,2) = 1;F(2,5) = 0.0501;
F(3,3) = 1;F(3,6) = 0.0501;
F(4,4) = 0.9426;
F(5,5) = 0.9378;
F(6,6) = 0.818;

G = [0; 0; 0; 0.06088; 0.06386; 0.1912];

v= [1, 1, 1, 0, 0, 0];
% v= [1, 0, 0, 0, 0, 0];
H = diag(v);
D =[0;0;0;0;0;0];

% Estimate Position and Velocity from the Given input cmd of Joy
EVk_x = Bdy.Velx;uv_x = setpoint.velx;
EVk_y = Bdy.Vely;uv_y = setpoint.vely;
EVk_z = Bdy.Velz;uv_z = setpoint.velz;
for j = 1 : size(setpoint.time, 1)
    Vk_x(j,1)=  vel_mesu_x(EVk_x(j,1), uv_x(j,1));
    Vk_y(j,1) = vel_mesu_y(EVk_y(j,1), uv_y(j,1));
    Vk_z(j,1) = vel_mesu_z(EVk_z(j,1), uv_z(j,1));
end

for j= 1: size(setpoint.time, 1)
    VinW(:,j) = velto_w(wld_qw(j,1), wld_qx(j,1), wld_qy(j,1), wld_qz(j,1), Vk_x(j,1), Vk_y(j,1), Vk_z(j,1));
    
end


for j = 1 : size(setpoint.time, 1)
    uv(:,j) = [0; 0; 0; uv_x(j,1); uv_y(j,1); uv_z(j,1)];
end



X_est = zeros([6, size(setpoint.time, 1)]);
X_est(1,1) = 1;
X_est(2,1) = -0.5;
X_est(3,1) = 0.6;
for j= 1 : size(setpoint.time, 1)
%     X_est(2,j) = Vk_z(j,1);
    X_est(4,j) = VinW(1,j);
    X_est(5,j) = VinW(2,j);
    X_est(6,j) = VinW(3,j);
end

for j = 2 : size(setpoint.time, 1)
    X_est(:, j) = F * X_est(:, j - 1) + G .* uv(:,j); %%uv_y(j-1,1);
end



% %%%%Building a System
% %sys_c=ss(A, B, C, D);
% %sys_d=c2d(sys_c, Ts, 'tustin');
sys_d = ss(F,G, H, 0, 0.0501);
% ss_d = idss(sys_d)

totf = tf(sys_d);
% sstotf = ss2tf(F,G,H,D,1)
tf_x = totf(1);
tf_y = totf(2);
tf_z = totf(3);

% % PID Tuning
% PID = pidtune(sys_d, 'pid');




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% %%%%%%PLOT THE DATA
figure
subplot(2,1,1)
hold on
plot(Bdy.Velx ,'R', 'LineWidth', 3)
plot(setpoint.velx, 'B', 'LineWidth', 2)
% compare(Bdy.Vely, setpoint.vely)
xlabel('Time_ Step (0.05)')
ylabel('Amplitude')
legend('Bdy_ Vx', 'Joy_ Inputs' ,'Location','northeast');
title('Bdy_ Vx & Joy_ Inputs')
hold off
subplot(2,1,2)
hold on
plot(X_est(1,:) ,'R', 'LineWidth', 3) %Time Vs Position of X
plot(Bdy_wrld.Posex , 'k', 'LineWidth', 2)
xlabel('Time_ Step (0.05)')
ylabel('Position (m)')
legend('Esti Pose_ X', 'Ture Pose_ X' ,'Location','northeast');
title('Esti Pose_ X & Ture Pose_ X' )
hold off
%%____________________________________
figure
subplot(2,1,1)
hold on
plot(Bdy.Vely ,'R', 'LineWidth', 3)
plot(setpoint.vely, 'B', 'LineWidth', 2)
xlabel('Time_ Step (0.05)')
ylabel('Amplitude')
legend('Bdy_ Vy', 'Joy_ Inputs' ,'Location','northeast');
title('Bdy_ Vy & Joy_ Inputs')
% compare(Bdy.Vely, setpoint.vely)
hold off
subplot(2,1,2)
hold on
plot(X_est(2,:) ,'R', 'LineWidth', 3) %Time Vs Position of X
plot(Bdy_wrld.Posey , 'K', 'LineWidth', 2)
xlabel('Time_ Step (0.05)')
ylabel('Position (m)')
legend('Esti Pose_ Y', 'Ture Pose_ Y' ,'Location','northeast');
title('Esti Pose_ Y & Ture Pose_ Y' )
hold off

%%____________________________________
figure
subplot(2,1,1)
hold on
plot(Bdy.Velz ,'R', 'LineWidth', 3)
plot(setpoint.velz, 'B', 'LineWidth', 2)
xlabel('Time_ Step (0.05)')
ylabel('Amplitude')
legend('Bdy_ Vz', 'Joy_ Inputs' ,'Location','northeast')
title('Bdy_ Vz & Joy_ Inputs')
hold off

subplot(2,1,2)
hold on
plot(X_est(3,:) ,'R', 'LineWidth', 3) %Time Vs Position of X
plot(Bdy_wrld.Posez , 'K', 'LineWidth', 2)
xlabel('Time_ Step (0.05)')
ylabel('Position (m)')
legend('Esti Pose_ Z', 'Ture Pose_ Z' ,'Location','northeast');
hold off
title('Esti Pose_ Z & Ture Pose_ Z' )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





%%%%% Function for Measurment%%%
%Function for  X
function Vk_x = vel_mesu_x(EVk_x, uv_x)  % y (measurment for given U input)
         Vk_x = (0.06088/(1-0.9426*EVk_x))*uv_x;        
end

%Function for  Y
function Vk_y = vel_mesu_y(EVk_y, uv_y)  % y (measurment for given U input)
         Vk_y = (0.06386/(1-0.9378*EVk_y))*uv_y;        
end

%Function for  Z
function Vk_z = vel_mesu_z(EVk_z, uv_z)  % y (measurment for given U input)
         Vk_z = (0.1912/(1-0.818*EVk_z))*uv_z;        
end

function VinW= velto_w(wld_qw, wld_qx, wld_qy, wld_qz, Vk_x, Vk_y, Vk_z)
    %%%Rotation Matrix
    quat = [wld_qw, wld_qx, wld_qy, wld_qz];
    eul = quat2eul(quat);
    %onlyyaw = [eul(3), eul(2), eul(1)];
    onlyyaw = [eul(3), 0, 0];
    R= eul2rotm(onlyyaw);             %%%%R_W_b
    Vw= R*[Vk_x; Vk_y; Vk_z]; %%%% Convert to World_V
    VinW=zeros(3,1);
    VinW(1,1) = Vw(1);  
    VinW(2,1) = Vw(2);
    VinW(3,1) = Vw(3);
end





