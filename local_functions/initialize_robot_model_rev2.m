function [robot, simulator_true] = initialize_robot_model_rev2(robot_type, models_path, pre_defined_path, regressor_selector)
   arguments
        robot_type = "kinova_gen3";
        models_path = "models";
        pre_defined_path = "./pre_defined/";
        regressor_selector = "BaseJointRegressor";
   end

   model = [];
   switch(robot_type)
       case "kinova_gen3"
            dof = 7;
            robot = kinova_gen3(model);
            robot.dof = dof;
       case "kuka_iiwa7"
           fprintf("Under developing\n");
       case "mecademic_meca500"
           dof = 6;
           robot = mecademic_meca500(model);
           robot.dof = dof;
           robot.regressor_selector = regressor_selector;
           
           if or( robot.regressor_selector==robot.regressor_selector_input{1}, ...
                  robot.regressor_selector==robot.regressor_selector_input{2})
                robot.num_of_links = 6;
           else
                robot.num_of_links = 7;
           end
           
           if( robot.dof ~= robot.num_of_links )
                base_name = "_inertial_geom_info_CAD_kai";
           else
                base_name = "_inertial_geom_info_CAD";
           end

       otherwise
           fprintf("Error! robot_type was not correct\n");
   end

    %% cad params 
    %tmp_q_dq_ddq = load(pre_defined_path+"motion_file.mat");

    % calc phib_true
    robot.load_model_file(models_path+robot_type+base_name);
    %robot.load_model_file(models_path+robot_type+"_inertial_geom_info_CAD_add_eff_weight");  % CAD model with **kg weight on end-effector
    robot.set_link_properties();    robot.set_inertial_param(); robot.set_root();
    %tmp_data.q = tmp_q_dq_ddq.R(:,1:6); tmp_data.dq = tmp_q_dq_ddq.R(:,8:13); tmp_data.ddq = tmp_q_dq_ddq.R(:,15:20);
    
    %[~,~,~, tmp_Y] = compose_regressor(tmp_data, robot);
    
    %robot.set_compose_X_Z_phib( tmp_Y );
    %phib_true = robot.phib;
    %phi_true = robot.phi;

    % calc phib_cad
    robot.load_model_file(models_path+robot_type+base_name); %CADmode (It is also used on Mujoco)
    %robot.load_model_file(models_path+robot_type+"_inertial_geom_info_CAD_add_eff_weight");  % CAD model with **kg weight on end-effector
    %robot.load_model_file(models_path+robot_type+"_inertial_geom_info_CAD_add_uncertainties_uniform_rand30%"); % CAD model + 30% uncertainties under unrand
    %robot.load_model_file(models_path+robot_type+"_inertial_geom_info_CAD_add_eff_weight_add_height");  % CAD model with **kg weight on end-effector
    %robot.load_model_file(models_path+robot_type+"_inertial_geom_info_CAD_add_uncertainties_uniform_rand30%_add_height"); % CAD model + 30% uncertainties under unrand
    
    robot.set_link_properties();    robot.set_inertial_param(); robot.set_root();
    %tmp_data.q = tmp_q_dq_ddq.R(:,1:6); tmp_data.dq = tmp_q_dq_ddq.R(:,8:13); tmp_data.ddq = tmp_q_dq_ddq.R(:,15:20);
    
    %[~,~,~, tmp_Y] = compose_regressor(tmp_data, robot);
    %robot.set_compose_X_Z_phib( tmp_Y );

    if 0
        robot.regressor_selector = robot.regressor_selector_input{4};
        t = [0:0.01:5]';
        offset = (-45*pi/180)*ones(size(t,1),1);
        sig = (20*pi/180)*sin(2*pi*0.4*t)+(-20*pi/180);
        dsig = (20*pi/180)*(2*pi*0.4)*cos(2*pi*0.4*t);
        ddsig = -(20*pi/180)*(2*pi*0.4).^2*sin(2*pi*0.4*t);

        data.q = [offset,zeros(501,1), sig, zeros(501,3)];
        data.dq = [zeros(501,2), dsig, zeros(501,3)];
        data.ddq = [zeros(501,2), ddsig, zeros(501,3)];
        
        [~,~,~, tmp_Y] = compose_regressor(data, robot);
        data.baseFT = reshape(tmp_Y*robot.phi, [], 6);

        ib = calc_ib(tmp_Y);
        X = conv_ib2X(ib,70);
        Z = calc_Z(X, tmp_Y);
        phib = Z*robot.phi;
        data.Z = Z;
        data.X = X;
        data.ib = ib;

        save("data_axis3_single_offset.mat", "data");

    end

    %% for base link regressor 
    %{
    tmp = load(pre_defined_path+"base_compose_mat.mat");

    robot.Zo_1 = tmp.base_compose_mat.Zo_1; 
    robot.Zo_2 = tmp.base_compose_mat.Zo_2;
    robot.X_base = tmp.base_compose_mat.X_base;

    robot.Yb0 = [];
    robot.Yb0T_Yb0 = 0;
    robot.F0 = [];

    Zo_1 = robot.Zo_1;
    Zo_2 = robot.Zo_2;

    simulator_true.phib = phib_true;
    simulator_true.phi = phi_true;
    %}
    simulator_true = [];
end