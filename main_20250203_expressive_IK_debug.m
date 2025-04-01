%% 2025 02 03 Author @ Kenya Mori

close all;
%clear all;
clearvars -except log_dir exp_loop tmp_use_q_derivative tmp_noise_enable tmp_lpf_enable params;
clc;

addpath("local_functions");
rng(0,"twister");

%% global variables
global hold_best_optim_coefs;
global best_optim_fval;

hold_best_optim_coefs = [];
best_optim_fval = 10E20;

%% flags
isdebug = 1;
use_WLS = 1;
use_EffMassModel = 0;
use_experiment_log = 1;
save_exci_traj = 1;
num_max_optim = 1;

forgetting_enable = 0;

robot_type = "mecademic_meca500";
regressor_selector = "BaseRegressor";

%% set dirs
%init_dirs;    
rpi_common_dir = "/home/pi/kenya/meca500_dev/workspace/";
session_dir = "20241016/06_500g_propAL_seed0";
actual_file_dir_core = "log/"+session_dir+"/";

%% robot model setting
[robot,~] = initialize_robot_model_rev2(robot_type, "models/", "./local_functions/", regressor_selector);
%robot = mecademic_meca500("mecademic_meca500");
robot.phib_ident = robot.phib';
robot.rsd_p_ident = ones(1, size(robot.phib,1) )*50.0;

%% twist for P1 => P2 with expressive movemnet

%P1 = [[0  130 242 ]/1000, [180 0 -90]*pi/180 ]; % user1
%P2 = [[130  0 242 ]/1000, [180 0 -90]*pi/180 ]; % user2
%P3 = [[0 -130 242 ]/1000, [180 0 -90]*pi/180 ]; % user3

check_path = "P3toP1"; %P1toP2, P3toP1
emotion = "sad5"; %joy, sad, sad2, neutral

if check_path == "P1toP2"
    %% P1 to P2
    t_max = 5; dt = 0.01;
    t = [0:dt:t_max]';
    freq = 1/(t_max*4);
    p_w_eff_x = 130/1000*sin(2*pi*freq*t);
    p_w_eff_y = 118.3216/1000*cos(2*pi*freq*t);
    p_w_eff_z_height = 213.7442/1000*ones(size(t,1),1);
    p_w_eff_z = (10/1000)*cos(2*pi*1*t) + p_w_eff_z_height - (10/1000);
    p_w_eff_y_ang = (10*pi/180)*sin(2*pi*2*t);
    init_ang = [ 90, -4.8936, -7.0517, -0.0, 92.1582, 90.0]'*pi/180;
else
    %% P3 to P1
    t_max = 10; dt = 0.01;
    t = [0:dt:t_max]';
    freq = 1/(t_max*2);
    p_w_eff_x = 130/1000*sin(2*pi*freq*t);
    p_w_eff_y = -118.3216/1000*cos(2*pi*freq*t);
    p_w_eff_z_height = 213.7442/1000*ones(size(t,1),1);
    init_ang = [ -90, -4.8936, -7.0517, -0.0, 92.1582, 90.0]'*pi/180;
    p_w_eff_y_ang = (10*pi/180)*sin(2*pi*2*t);
end

if emotion=="joy"
    p_w_eff_z = (10/1000)*cos(2*pi*1*t) + p_w_eff_z_height - (10/1000);
    %p_w_eff_y_ang = (10*pi/180)*sin(2*pi*1*t);
elseif emotion=="sad"
    p_w_eff_z = p_w_eff_z_height - 0.1*t/t_max;
    %p_w_eff_y_ang = (10*pi/180)*sin(2*pi*1*t);
elseif emotion=="sad2"
    p_w_eff_z = p_w_eff_z_height + abs( (50/1000)*sinc(2*pi*freq*3*t)) - (50/1000);
elseif emotion=="neutral"
    p_w_eff_z = p_w_eff_z_height;
elseif emotion=="sad3" % sin - kt/t_max
    p_w_eff_z = p_w_eff_z_height -(15/1000)+ abs( (15/1000)*cos(2*pi*freq*t)) + (-30/1000)*t/t_max;
elseif emotion=="sad4" % abs(cos) - kt/t_max
    p_w_eff_z = p_w_eff_z_height -(15/1000)+ abs( (15/1000)*cos(2*pi*freq*4*t)) + (-30/1000)*t/t_max;
elseif emotion=="sad5" % abs(cos).*exp(-v*t/t_max) - kt/t_max
    p_w_eff_z = p_w_eff_z_height -(20/1000)+ abs( (20/1000)*cos(2*pi*freq*4*t)).*exp(-3*t/t_max) + (-30/1000)*t/t_max;
elseif emotion=="sad6" %step
    %p_w_eff_z = p_w_eff_z_height + abs( (50/1000)*sinc(2*pi*freq*3*t)) - (50/1000);
    p_w_eff_z = zeros(size(t));
    size_t = size(p_w_eff_z,1);
    delta_z = 15;
    for i=2:size_t
        if i<size_t/4
            p_w_eff_z(i) = p_w_eff_z_height(1) - delta_z;
        elseif and( size_t/4<=i , i<2*size_t/4)
            p_w_eff_z(i) = p_w_eff_z_height(1) - 2*delta_z;
        elseif and(2*size_t/4<=i,i<3*size_t/4)
            p_w_eff_z(i) = p_w_eff_z_height(1) - 3*delta_z;
        elseif i>=3*size_t/4
            p_w_eff_z(i) = p_w_eff_z_height(1) - 4*delta_z;
        end
    end
end

% normal test condition
p_w_eff = [p_w_eff_x, p_w_eff_y, p_w_eff_z_height, zeros(size(t,1),1), zeros(size(t,1),1), zeros(size(t,1),1) ];
p_w_eff2 = [zeros(size(t,1),2),p_w_eff_z, zeros(size(t,1),1), p_w_eff_y_ang, zeros(size(t,1),1) ];

% only orientation
%p_w_eff = [zeros(size(t,1),4), p_w_eff_y_ang, zeros(size(t,1),1) ];
%p_w_eff2 = [zeros(size(t,1),2),p_w_eff_z, zeros(size(t,1),1), p_w_eff_y_ang, zeros(size(t,1),1) ];

% ***** test condition
%p_w_eff = [p_w_eff_x, p_w_eff_y, p_w_eff_z, zeros(size(t,1),1), p_w_eff_y_ang, zeros(size(t,1),1) ];
%p_w_eff2 = [zeros(size(t,1),2),p_w_eff_z, zeros(size(t,1),1), p_w_eff_y_ang, zeros(size(t,1),1) ];


twist_v2 = gradient(p_w_eff')'/dt;
twist_v2_2 = gradient(p_w_eff2')'/dt;

%{
R_cmp_eff_w = [0 -1 0; -1 0 0; 0 0 -1];
A_cmp_eff_w = [R_cmp_eff_w, zeros(3,3); zeros(3,3), R_cmp_eff_w];
%}

p_w_eff_integ = zeros(size(p_w_eff,1), size(p_w_eff,2));
p_w_eff_integ(1,:) = p_w_eff(1,:);
for k=2:size(t,1)
    p_w_eff_integ(k,:) = p_w_eff_integ(k-1,:) + twist_v2(k,:)*dt;
end

twist_v3 = zeros(size(t,1), 6);

q = init_ang;
dq_ref_old = zeros(6,1);
q_ref_v_em = zeros(size(t,1),6);
dq_ref_v_em = zeros(size(t,1),6);
twist_ref_v = zeros(size(t,1),6);
twist_ref_v2 = zeros(size(t,1),6);
p_eff_em = zeros(size(t,1),6);

rank_v = zeros(size(t,1),2);

for k=1:size(t,1)
    
    [p,R,~] = robot.calc_fk(q);
    R_w_eff = reshape( R(end,:),3,3 );
    if k==1
        R_w_eff_old = R_w_eff;
    end
    dR_w_eff = (R_w_eff - R_w_eff_old)/dt;
    w_w_eff = cross_x(dR_w_eff*R_w_eff');

    twist_w_eff = twist_v2(k,:)';
    twist_w_eff_2 = twist_v2_2(k,:)';
    
    twist_eff_eff = [R_w_eff, zeros(3,3); zeros(3,3), R_w_eff]'*twist_w_eff;
    twist_eff_eff_2 = [R_w_eff, zeros(3,3); zeros(3,3), R_w_eff]'*twist_w_eff_2;
    
    J = robot.calc_eff_jacobian2(q);
    
    %{
    %% plan1
    
    J2 = [J(1,:); J(2,:); J(4,:); J(6,:) ];
    %J2 = [J(3,:) ];
    
    J2_psu = J2'*inv(J2*J2' + eye(size(J2,1))*0.00001);
    J2_Ipsu = ( eye(6)-J2_psu*J2 );
    
    J3 = [J(3,:);J(5,:); ];
    %J3 = [J(1,:); J(2,:); J(4,:); J(5,:); J(6,:) ];
    J3_psu = J3'*inv(J3*J3' + eye(size(J3,1))*0.00001);
    
    J3_bar = J3*J2_Ipsu;
    J3_bar_psu = J3_bar'*inv(J3_bar*J3_bar' + eye(size(J3_bar,1))*0.00001);

    task_main = twist_eff_eff([1,2,4,6],1);
    task_sub = twist_eff_eff_2([3,5],1);
    dq_ref = J2_psu*task_main + J2_Ipsu*J3_psu*task_sub;
    %dq_ref = J2_psu*task_main + J3_bar_psu*( task_sub - J3*J2_psu*task_main );
    %}

    %{
    %% plan2
    J2 = J;
    J2_psu = J2'*inv(J2*J2' + eye(6)*0.00001);
    J2_Ipsu = ( eye(6)-J2_psu*J2 );
    %J2_psu = inv(J2'*J2 + eye(6)*0.00001)' * J2';
    
    task_main = [twist_eff_eff(1);twist_eff_eff(2);0; twist_eff_eff(4);0;twist_eff_eff(6)];
    task_sub = [0;0;twist_eff_eff_2(3); 0;twist_eff_eff_2(5);0];
    dq_ref = J2_psu*task_main + J2_Ipsu*task_sub;
    %}

    %{
    %% plan3
    J2 = (J'*diag([1,1,0,1,1,1]))';
    J2_psu = J2'*inv(J2*J2' + eye(6)*0.000001); %SR inverse
    %J2_psu = pinv(J2);
    J2_Ipsu = ( eye(6)-J2_psu*J2 );
    %J2_psu = inv(J2'*J2 + eye(6)*0.00001)' * J2';
    
    J3 = (J'*diag([0,0,1,0,0,0]))';
    J3_psu = J3'*inv(J3*J3' + eye(6)*0.000001); %SR inverse
    J3_Ipsu = ( eye(6)-J3_psu*J3 );

    J3_bar = J3*J2_Ipsu;
    J3_bar_psu = J3_bar'*inv(J3_bar*J3_bar' + eye(size(J3_bar,1))*0.00001);

    task_main = twist_eff_eff;
    task_sub = twist_eff_eff_2; %[0;0;twist_eff_eff_2(3); 0;twist_eff_eff_2(5);0];
    
    %dq_ref = J2_psu*task_main + J2_Ipsu*J3_psu*task_sub;
    %dq_ref = lsqminnorm(J2, task_main);
    dq_ref = J2_psu*task_main + J3_bar_psu*( task_sub - J3*J2_psu*task_main );
    %}

    %% plan4
    selection_vec = [1,1,0,1,1,1];
    [dq_ref,J2,twist_w_eff_ref] = calc_dq_ref( robot, q, twist_w_eff, twist_w_eff_2, selection_vec,dt );
    
    q = q + (dq_ref + dq_ref_old)*dt*0.5;
    dq_ref_old = dq_ref;

    R_w_eff_old = R_w_eff;

    q_ref_v_em(k,:) = q';
    dq_ref_v_em(k,:) = dq_ref';
    twist_ref_v(k,:) = twist_eff_eff';
    twist_ref_v2(k,:) = twist_eff_eff_2';
    p_eff_em(k,:) = [ p(end,:), w_w_eff'];
    rank_v(k,:) = [rank(J2), cond(J2)];

    twist_v3(k,:) = twist_w_eff_ref';
end
tmp_tw = [twist_ref_v(:,1),twist_ref_v(:,2),twist_ref_v2(:,3),twist_ref_v(:,[4:6])];

writematrix([twist_ref_v,twist_ref_v2], "./motions/motion_offline_ik_3_hw.csv");
writematrix(tmp_tw, "./motions/motion_offline_ik_3_hw_debug.csv");
writematrix(dq_ref_v_em, "./motions/motion_offline_ik_3_dq.csv");

figure(1); hold on ;
sgtitle("null space IK result: time series pos xyz");
tmp = [p_w_eff(:,1), p_w_eff(:,2), p_w_eff2(:,3)];
for i=1:3
    subplot(3,1,i); hold on; grid;
    plot(tmp(:,i), "r"); plot(p_eff_em(:,i), "b--");
    plot(tmp(:,i)-p_eff_em(:,i), "g");
end
legend("Ref.","fk result", "err");

figure(2); hold on ; grid;
sgtitle("null space IK result: task space traj.");
plot3(tmp(:,1), tmp(:,2), tmp(:,3), "r");
plot3(p_eff_em(:,1), p_eff_em(:,2),p_eff_em(:,3), "b--");
legend("Ref.","fk result");
axis([-0.02, 0.14, -0.14, 0.14, 0,0.5]);


figure(3); hold on ;
sgtitle("null space IK result: ts twist");
tmp2 = [twist_v2(:,4), twist_v2_2(:,5), twist_v2(:,6)];
for i=1:3
    subplot(3,1,i); hold on; grid;
    plot(tmp2(:,i), "r"); plot(p_eff_em(:,i+3), "b--");
end
legend("Ref.","fk result");

function [dq_ref,J2,twist_w_eff_ref] = calc_dq_ref( robot, q, twist_w_eff, twist_w_eff_2, selection_vec,dt )
    twist_w_eff_tmp = twist_w_eff.*selection_vec';
    twist_w_eff_2_tmp = twist_w_eff_2.*((selection_vec-1)*-1)';
    dq_ref = zeros(6,1);

    for k=1:1

        [p,R,~] = robot.calc_fk(q);
        p_w_w_eff = p(end,:)';
        R_w_eff = reshape( R(end,:),3,3 );
        
        A_vel_w_eff = [R_w_eff, zeros(3,3); zeros(3,3), R_w_eff];
        
        J = robot.calc_eff_jacobian2(q);
        
        if k==1
            twist_eff_eff   = A_vel_w_eff'*twist_w_eff_tmp;
            twist_eff_eff_2 = A_vel_w_eff'*twist_w_eff_2_tmp;
        else
            %twist_w_eff_tmp   = twist_w_eff_tmp + ( twist_w_eff - A_vel_w_eff*(J*dq_ref) );
            %twist_eff_eff = A_vel_w_eff'*twist_w_eff_tmp;
            %twist_w_eff_2_tmp   = twist_w_eff_2_tmp + ( twist_w_eff_2 - A_vel_w_eff*(J*dq_ref) );
            %twist_eff_eff_2 = A_vel_w_eff'*twist_w_eff_2_tmp;

        end
        
        J2 = (J'*diag( selection_vec ))';
        J2_psu = J2'*inv(J2*J2' + eye(6)*0.000001); %SR inverse
        %J2_psu = pinv(J2);
        J2_Ipsu = ( eye(6)-J2_psu*J2 );
        %J2_psu = inv(J2'*J2 + eye(6)*0.00001)' * J2';
    
        J3 = (J'*diag((selection_vec-1)*-1))';
        J3_psu = J3'*inv(J3*J3' + eye(6)*0.000001); %SR inverse
        J3_Ipsu = ( eye(6)-J3_psu*J3 );

        J3_bar = J3*J2_Ipsu;
        J3_bar_psu = J3_bar'*inv(J3_bar*J3_bar' + eye(size(J3_bar,1))*0.00001);

        task_main = twist_eff_eff;
        task_sub = twist_eff_eff_2;
    
        %dq_ref = J2_psu*task_main + J2_Ipsu*(J3_psu*task_sub);
        %dq_ref = lsqminnorm(J2, task_main);
        dq_ref = J2_psu*task_main + J3_bar_psu*( task_sub - J3*J2_psu*task_main );

    end
    twist_w_eff_ref = A_vel_w_eff*(J*dq_ref);
end