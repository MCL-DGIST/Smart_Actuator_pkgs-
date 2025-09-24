clear; clc; close all;

% figure(n)
% print('-clipboard', '-dbitmap')  // ubuntu figure 복사 

start_time = 0; % 시작 시간
end_time = 1;   % 끝 시간

pos_plot_flag = true;
vel_plot_flag = true;
Body_state_plot_flag = true;


x_pos_lb = -0.5; x_pos_ub = 0.5;
y_pos_lb = -0.5; y_pos_ub = 0.5;
z_pos_lb = -0.5; z_pos_ub = 0;

x_vel_lb = -2; x_vel_ub = 2;
y_vel_lb = -2; y_vel_ub = 2;
z_vel_lb = -2; z_vel_ub = 2;




tau_lb = -40; tau_ub = 40;

xyz_label = {'$x\ \mathrm{(m)}$', '$y\ \mathrm{(m)}$', '$z\ \mathrm{(m)}$'};
opt_GRF_label = {'$F_x\ \mathrm{(N)}$', '$F_y\ \mathrm{(N)}$', '$F_z\ \mathrm{(N)}$'};
rpy_label = {'$\alpha\ \mathrm{(rad)}$', '$\beta\ \mathrm{(rad)}$', '$\gamma\ \mathrm{(rad)}$'};
xyzdot_label = {'$\dot{x}\ \mathrm{(m/s)}$', '$y\ \mathrm{(m/s)}$', '$z\ \mathrm{(m/s)}$'};
rpydot_label = {'$\alpha\ \mathrm{(rad/s)}$', '$\beta\ \mathrm{(rad/s)}$', '$\gamma\ \mathrm{(rad/s)}$'};


xyz_lb = {-0.2, -0.2, 0};   xyz_ub = {0.2, 0.2, 0.5};
rpy_lb = {-0.1, -0.1, -0.1};   rpy_ub = {0.1, 0.1, 0.1};
xyzdot_lb = {-0.3, -0.3, -0.3};   xyzdot_ub = {0.3, 0.3, 0.3};
rpydot_lb = {-0.3, -0.3, -0.3};   rpydot_ub = {0.3, 0.3, 0.3};

filename{1} = '/home/jinsong/Desktop/mcl_quadruped_pkgs/mujoco/mujoco-3.3.1/One-Leg/data/data.csv';
% filename{2} = '/home/jinsong/Desktop/mujoco_cpp/mujoco-3.1.6/myproject/Quad_v2/Quad_DDP/data/data_FR.csv';
% filename{3} = '/home/jinsong/Desktop/mujoco_cpp/mujoco-3.1.6/myproject/Quad_v2/Quad_DDP/data/data_RL.csv';
% filename{4} = '/home/jinsong/Desktop/mujoco_cpp/mujoco-3.1.6/myproject/Quad_v2/Quad_DDP/data/data_RR.csv';
filename_Body = '/home/jinsong/Desktop/mujoco_cpp/mujoco-3.1.6/myproject/Quad_v2/Quad_DDP/data/data_Body.csv';

    
% for i = 1:1:4
    % Arr_Leg{i} = table2array(readtable(filename{i}));
% end

Arr_Leg = table2array(readtable(filename{1}));

t           = Arr_Leg(:,1);

pos_x = Arr_Leg(:,2);
pos_z = Arr_Leg(:,3);
pos_x_ref = Arr_Leg(:,4);
pos_z_ref = Arr_Leg(:,5);
vel_x = Arr_Leg(:,6);
vel_z = Arr_Leg(:,7);
vel_body_z = Arr_Leg(:,8);


Arr_Body = table2array(readtable(filename_Body));

% [m1,n1] = size(Arr_FL);
% [m2,n2] = size(Arr_Body);

% 
%Body state estimation

x0 = Arr_Body(:,1:12); 
x_ref = Arr_Body(:,13:24);




%%%%%%%%%%%%%%%% Data Ploting %%%%%%%%%%%%%%%%%%%%%%%%%
%time cutting
% 시작 시간과 끝 시간 설정

% t 벡터를 기준으로 시작 시간과 끝 시간에 해당하는 인덱스 찾기
s_idx = find(t >= start_time, 1);
e_idx = find(t >= end_time, 1);
t = t(s_idx:e_idx);


%Data ploting 관련 Parameter
lw =1.2;   %Line Width
LW = 1.7;
Title_size = 18; %Title Fonte Size
Axis_size = 12; %Axis Fonte Size
legend_size =8 ; % Legend Fonte Size


n = 1;

if pos_plot_flag == true
    
        figure(n); n = n+1;       
        
        subplot(2,1,1)
        plot(t(s_idx:e_idx),pos_x_ref(s_idx:e_idx),'black--','LineWidth', LW);
        hold on
        plot(t(s_idx:e_idx),pos_x(s_idx:e_idx),'r-','LineWidth',lw);
        grid on;
            legend('ref','act','FontName','Times New Roman','location','northeast','FontSize',10,'Interpreter', 'latex')
        xlabel('Time (seconds)','FontSize', Axis_size);
        ylabel('x Position (m)','FontSize', Axis_size);
        ylim([x_pos_lb x_pos_ub]);

        subplot(2,1,2)
        plot(t(s_idx:e_idx),pos_z_ref(s_idx:e_idx),'black--','LineWidth', LW);
        hold on
        plot(t(s_idx:e_idx),pos_z(s_idx:e_idx),'r-','LineWidth',lw);
        grid on;
        xlabel('Time (seconds)','FontSize', Axis_size);
        ylabel('z Position (m)','FontSize', Axis_size);
        ylim([z_pos_lb z_pos_ub]);


        sgtitle("Position",'FontName','times new roman', 'Fontsize', Title_size);
             
end

if vel_plot_flag == true
    
        figure(n); n = n+1;       
        
        subplot(2,1,1)
        plot(t(s_idx:e_idx),vel_x(s_idx:e_idx),'r-','LineWidth', LW);
        hold on;
        grid on;
            legend('act','FontName','Times New Roman','location','northeast','FontSize',10,'Interpreter', 'latex')
        xlabel('Time (seconds)','FontSize', Axis_size);
        ylabel('x Velocity (m/s)','FontSize', Axis_size);
        

        subplot(2,1,2)
        plot(t(s_idx:e_idx),vel_z(s_idx:e_idx),'r-','LineWidth', LW);
        hold on;
        grid on;
        xlabel('Time (seconds)','FontSize', Axis_size);
        ylabel('z Velocity (m/s)','FontSize', Axis_size);


        sgtitle("Velocity",'FontName','times new roman', 'Fontsize', Title_size);
           
        
end



if Body_state_plot_flag == true
        
        figure(n); n = n+1; 
        
            plot(t(s_idx:e_idx),vel_body_z(s_idx:e_idx),'r-','LineWidth', LW);
            hold on;
            % ylabel(filename_Body,'FontSize', Axis_size, 'Interpreter', 'latex');
            legend('Actual','FontName','Times New Roman','location','northeast','FontSize',10,'Interpreter', 'latex')
            grid on;
        
            sgtitle("Body z Velocity",'FontName','Times New Roman', 'Fontsize', Title_size);

end

setFigurePositions(3);

