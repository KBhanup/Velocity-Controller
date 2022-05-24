%% Clear everything
clear; close all; clc;

% Load Input, OutPut, and TransferFn data
load('Z_Joy_cmds.mat');
load('Z_Sel_Pose_Z.mat');
load('Z_V_Z_bodyVel.mat');
load('Zwrld_qw.mat');
load('Zwrld_qx.mat');
load('Zwrld_qy.mat');
load('Zwrld_qz.mat');

%%%%%%%%%%%%%%%---------------Checking PID Tunes--------------%%%%%%%%%%%%%
% e_k = Error @ time step K
% e_k-1= e_p = Error @ time step k-1 (earlier time step)
% Kp, Kd = Proportional and Differential gains
% U = input
% P_d = desired position
% P_c = current position
% F,G,H Sate Dynamics

% %Creating a Transfer Function Model

%%% State Space System only for Z axis

% Define System Dynamics F,G
F =zeros([2,2]);
F(1,1) = 1;F(1,2) = 0.0501;
F(2,2) = 0.818;

G = [0; 0.1912];

v= [1, 0];
H = diag(v);
D =[0;0];

% From setpoints time we can calc dt=t2-t1;
dt=0.0501;

%P_d = 0.6;
P_d = ZSel_Pose_Z;
P_C(1)=0.8;   %%%P_C(1) is the Initial Current position
e_p(1) =0;
Kp = 1.3131;%1.3131;
Kd = 0.32893;%0.032893;
X_est = zeros([2, size(ZSel_Pose_Z, 1)]);
X_est(1,1)=0.8;     %%Inital estimated position
for j= 2:size(ZSel_Pose_Z)
    %e_k(j-1,1) = P_d - P_C(1,j-1);
    e_k(j-1,1) = P_d(j-1,1) - P_C(1,j-1);
    e_p(j,1) = e_k(j-1,1);
    U(j-1,1) = input_mesu(e_k(j-1,1), e_p(j-1,1), Kp, Kd);
    %Limit the input between -0.9 to +0.9
        if U(j-1,1)>= 1
            U(j-1,1)= 0.99;
        end
        if U(j-1,1)<= -1
            U(j-1,1)= -0.99;
        end
    Vk_z(j-1,1) = vel_mesu_z(ZV_Z_BodyVel(j-1,1), U(j-1,1));
    VinW(1,j-1) = velto_w(Zwrld_qw(j-1,1), Zwrld_qx(j-1,1), Zwrld_qy(j-1,1), Zwrld_qz(j-1,1), Vk_z(j-1,1));
    X_est(2,j-1) = VinW(1,j-1);
    X_est(:, j) = F * X_est(:, j - 1) + G .* U(j-1, 1); %%% Check for U j-1 or j???
    P_C(j) = X_est(1,j-1);

end
P_C = P_C';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% %%%%%%PLOT THE DATA
figure
subplot(2,1,1)
hold on
plot(U ,'R', 'LineWidth', 3)
plot(ZJoy_cmd_Z, 'B', 'LineWidth', 2)
% ylim([-1,1])
% compare(Bdy.Vely, setpoint.vely)
xlabel('Time_ Step (0.05)')
ylabel('Amplitude')

legend('PD-Tuned Input', 'Manual-Joy_ Inputs' ,'Location','northeast');
title('Joy_ Inputs actual Vs Manual')
hold off

subplot(2,1,2)
hold on
plot(X_est(1,:) ,'R', 'LineWidth', 3) %Time Vs Position of X
plot(ZSel_Pose_Z , 'B', 'LineWidth', 2)
% ylim([-1.2,1.2])
xlabel('Time_ Step (0.05)')
ylabel('Position (m)')
legend('Esti Pose_ Z with New Input', 'True Pose_ Z-Maual Input' ,'Location','northeast');
title('Esti Pose_ Z & True Pose_ Z' )
hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Function for Measurment%%%

function U = input_mesu(e_k, e_p, Kp, Kd)
         U = e_k*Kp + (e_k-e_p)*Kd;
end


%Function for  Z
function Vk_z = vel_mesu_z(ZV_Z_BodyVel, U)  % This would be in Body frame y (measurment for given U input)
         Vk_z = (0.1912/(1-0.818*ZV_Z_BodyVel))*U;        
end

function VinW= velto_w(Zwrld_qw, Zwrld_qx, Zwrld_qy, Zwrld_qz, Vk_z)
    %%%Rotation Matrix
    quat = [Zwrld_qw, Zwrld_qx, Zwrld_qy, Zwrld_qz];
    eul = quat2eul(quat);
    onlyyaw = [eul(3), eul(2), eul(1)];
    %onlyyaw = [eul(3), 0, 0];
    R= eul2rotm(onlyyaw);             %%%%R_W_b
    Vw= R*[0; 0; Vk_z]; %%%% Convert to World_V
    VinW=zeros(1,1);
%     VinW(1,1) = Vw(1);  
%     VinW(2,1) = Vw(2);
    VinW(1,1) = Vw(3);
end
