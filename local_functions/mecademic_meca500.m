classdef mecademic_meca500
%% 2024 05 26 new 
%  added L0 consideration & num_of_links

    properties (Access = public)
        dof;
        num_of_links;
        regressor_selector;
    end
    properties (Dependent)
        phi;
        link;
        root;
        phib;
        phib_ident;
        X;
        Z;
        Yall_AL;
        pos_all;
        baseY_all;

        baseY0;
        baseFT0;
        Zo_1;
        Zo_2;
        X_base;

        tau_limit; %Nm
        tau_limit_adaptive; %Nm
        ang_limit; %rad
        vel_limit; %rad/s
        acc_limit; %rad/s^2
        CoP_limit; % m
        last_coef;
        ib;
        Yb0;
        Yb0T_Yb0;
        F0;
        rsd_p_ident;
    end
    properties (Constant)
       g = 9.81;
       control_freq = 100;
       data = mecademic_meca500_sharedData;
       regressor_selector_input = { ...
       "JointRegressor", ...
       "BaseJointRegressor_L1L6", ...
       "BaseJointRegressor", ...
       "BaseRegressor", ...
       };
    end
    
    methods
        function obj = mecademic_meca500(d)
        end
        
        function value = get.phi(obj)
            value = obj.data.phi;
        end
        function value = get.link(obj)
            value = obj.data.link;
        end
        function value = get.root(obj)
            value = obj.data.root;
        end
        function value = get.phib(obj)
            value = obj.data.phib;
        end
        function value = get.X(obj)
            value = obj.data.X;
        end
        function value = get.Z(obj)
            value = obj.data.Z;
        end
        function value = get.Yall_AL(obj)
            value = obj.data.Yall_AL;
        end
        function value = get.pos_all(obj)
            value = obj.data.pos_all;
        end
        function value = get.baseY_all(obj)
            value = obj.data.baseY_all;
        end
        function value = get.baseY0(obj)
            value = obj.data.baseY0;
        end
        function value = get.baseFT0(obj)
            value = obj.data.baseFT0;
        end

        function value = get.Zo_1(obj)
            value = obj.data.Zo_1;
        end
        function value = get.Zo_2(obj)
            value = obj.data.Zo_2;
        end
        function value = get.X_base(obj)
            value = obj.data.X_base;
        end

        function value = get.tau_limit(obj)
            value = obj.data.tau_limit;
        end
        function value = get.tau_limit_adaptive(obj)
            value = obj.data.tau_limit_adaptive;
        end
        function value = get.ang_limit(obj)
            value = obj.data.ang_limit;
        end
        function value = get.vel_limit(obj)
            value = obj.data.vel_limit;
        end
        function value = get.acc_limit(obj)
            value = obj.data.acc_limit;
        end
        function value = get.CoP_limit(obj)
            value = obj.data.CoP_limit;
        end
        
        function value = get.last_coef(obj)
            value = obj.data.last_coef;
        end
        function value = get.ib(obj)
            value = obj.data.ib;
        end
        function value = get.Yb0(obj)
            value = obj.data.Yb0;
        end
        function value = get.Yb0T_Yb0(obj)
            value = obj.data.Yb0T_Yb0;
        end
        function value = get.F0(obj)
            value = obj.data.F0;
        end
        function value = get.rsd_p_ident(obj)
            value = obj.data.rsd_p_ident;
        end
        function value = get.phib_ident(obj)
            value = obj.data.phib_ident;
        end

        function obj = set.link(obj, value)
            obj.data.link = value;
        end
        function obj = set.root(obj, value)
            obj.data.root = value;
        end
        function obj = set.Yall_AL(obj, value)
            obj.data.Yall_AL = value;
        end
        function obj = set.pos_all(obj, value)
            obj.data.pos_all = value;
        end
        function obj = set.baseY_all(obj, value)
            obj.data.baseY_all = value;
        end
        function obj = set.baseFT0(obj, value)
            obj.data.baseFT0 = value;
        end
        function obj = set.baseY0(obj, value)
            obj.data.baseY0 = value;
        end

        function obj = set.Zo_1(obj, value)
            obj.data.Zo_1 = value;
        end
        function obj = set.Zo_2(obj, value)
            obj.data.Zo_2 = value;
        end
        function obj = set.X_base(obj, value)
            obj.data.X_base = value;
        end

        function obj = set.last_coef(obj, value)
            obj.data.last_coef = value;
        end

        function obj = set.tau_limit(obj, value)
            obj.data.tau_limit = value;
        end
        function obj = set.tau_limit_adaptive(obj, value)
            obj.data.tau_limit_adaptive = value;
        end
        function obj = set.ang_limit(obj, value)
            obj.data.ang_limit = value;
        end
        function obj = set.vel_limit(obj, value)
            obj.data.vel_limit = value;
        end
        function obj = set.acc_limit(obj, value)
            obj.data.acc_limit = value;
        end
        function obj = set.CoP_limit(obj, value)
            obj.data.CoP_limit = value;
        end

        function obj = set.Yb0(obj, value)
            obj.data.Yb0 = value;
        end
        function obj = set.Yb0T_Yb0(obj, value)
            obj.data.Yb0T_Yb0 = value;
        end
        function obj = set.F0(obj, value)
            obj.data.F0 = value;
        end
        function obj = set.rsd_p_ident(obj, value)
            obj.data.rsd_p_ident = value;
        end
        function obj = set.phib_ident(obj, value)
            obj.data.phib_ident = value;
        end

        function obj = load_model_file( obj, filename )

            T = readtable(filename+".xlsx");

            obj.data.cad(:,1) = T.m;    obj.data.cad(:,2) = T.cx;   obj.data.cad(:,3) = T.cy;   obj.data.cad(:,4) = T.cz;
            
            obj.data.cad(:,5) = T.fullinertia11;        obj.data.cad(:,6) = T.fullinertia22;
            obj.data.cad(:,7) = T.fullinertia33;        obj.data.cad(:,8) = T.fullinertia12;
            obj.data.cad(:,9) = T.fullinertia13;        obj.data.cad(:,10) = T.fullinertia23;
            obj.data.cad(:,11) = zeros(obj.num_of_links,1);
            %{
            obj.data.cad(:,5) = T.InertialFrameQuat1;   obj.data.cad(:,6) = T.InertialFrameQuat2;
            obj.data.cad(:,7) = T.InertialFrameQuat3;   obj.data.cad(:,8) = T.InertialFrameQuat4;
            obj.data.cad(:,9)  = T.diagInertiaX;
            obj.data.cad(:,10) = T.diagInertiaY;
            obj.data.cad(:,11) = T.diagInertiaZ;
            %}
            
            obj.data.cad(:,12) = T.pjjiX;       obj.data.cad(:,13) = T.pjjiY;       obj.data.cad(:,14) = T.pjjiZ;
            obj.data.cad(:,15) = T.selectorX;   obj.data.cad(:,16) = T.selectorY;   obj.data.cad(:,17) = T.selectorZ;
            %obj.data.cad(:,18) = T.rotZ1;       obj.data.cad(:,19) = T.rotY1;       obj.data.cad(:,20) = T.rotZ2;
            obj.data.cad(:,21) = T.rotQuat1;    obj.data.cad(:,22) = T.rotQuat2;
            obj.data.cad(:,23) = T.rotQuat3;    obj.data.cad(:,24) = T.rotQuat4;
            
        end
        function obj = load_joint_limits_file( obj, filename )

            T = readtable(filename+".xlsx");

            obj.data.ang_limit = deg2rad([T.angMin, T.angMax]);
            obj.data.vel_limit = deg2rad([T.velMax]);
            obj.data.acc_limit = deg2rad([T.accMax]);
            obj.data.tau_limit = [T.tauMax];
            
        end

        function obj = set_link_properties( obj )

            if isempty(obj.dof)
                fprintf("Error variable dof is empty\n");
                return;
            end
            if isempty(obj.data.cad)
                fprintf("Error variable data_cad is empty\n");
                return;
            end

            start_idx = 1;
            end_idx = 6;
            if obj.num_of_links ~= obj.dof
                %start_idx = 2;
                end_idx = end_idx+1;
            end

            for k=start_idx:end_idx

                obj.data.link(k).p_j_ji     = [obj.data.cad(k,12:14)]';
                obj.data.link(k).selector   = [obj.data.cad(k,15:17)]';
                %obj.data.link(k).R_base_j_i = rotz( obj.data.cad(k,18) )*roty( obj.data.cad(k,19) )*rotz( obj.data.cad(k,20) );
                
                quat = obj.data.cad(k,21:24);
                qVec = quaternion( quat );
                obj.data.link(k).R_base_j_i = rotmat(qVec,'frame')';

                obj.data.link(k).q_i   = 0;
                obj.data.link(k).dq_i  = 0;
                obj.data.link(k).ddq_i = 0;
                obj.data.link(k).R_j_i = eye(3);
            end
        end

        %% need to update to consider the L0 link
        function obj = load_link_collision_sphere_file( obj, filename )
            
            if isempty(obj.dof)
                fprintf("Error variable dof is empty\n");
                return;
            end
            
            T = readtable(filename+".xlsx");
            for k=1:obj.dof

                obj.data.link(k).sphere_center = [T.sphereX(k), T.sphereY(k), T.sphereZ(k)]';
                obj.data.link(k).sphere_radius = T.sphereR(k);
            end
            
            
        end

        function obj = set_inertial_param( obj )
            
            if isempty(obj.dof)
                fprintf("Error variable dof is empty\n");
                return;
            end
            if isempty(obj.data.cad)
                fprintf("Error variable data_cad is empty\n");
                return;
            end

            phi = [];
            for i=1:obj.num_of_links
                quat = obj.data.cad(i,5:8);
                qVec = quaternion( quat );
                R_i_iI = rotmat(qVec,'frame');              % the rotation matrix from the i-frame(Sigma_i) to the i-inertial frame(Sigma_iI)
    
                mi      = obj.data.cad(i,1);
                p_i_ic  = obj.data.cad(i,2:4)';             % the distance from the origin of the i-frame(Sigma_i) to the CoM, in the i-frame(Sigma_i)
                
                %kinova gen3
                %I_iI_Ic = diag( obj.data.cad(i,9:11) );     % the inertial matrix centered at the CoM, in the i-inertial frame(Sigma_iI)
                %I_i_Ic  = (R_i_iI'*I_iI_Ic*R_i_iI);         % the inertial matrix centered at the CoM, in the i-frame(Sigma_i)
                
                %mecademic meca500
                I_i_Ic = [obj.data.cad(i,5), obj.data.cad(i,8), obj.data.cad(i,9); ...
                          obj.data.cad(i,8), obj.data.cad(i,6), obj.data.cad(i,10); ...
                          obj.data.cad(i,9), obj.data.cad(i,10),obj.data.cad(i,7)];

                %% for debug
                if 0
                    
                    [tmp_R_i_iI2, tmp_I_iI_Ic2,tmp_R_i_iI2_] = svd(I_i_Ic); 
                    
                    R1 = R_i_iI;
                    R2 = tmp_R_i_iI2';

                    tmp_quat_1 = quaternion(R1,"rotmat","frame");
                    [q_a,q_b,q_c,q_d]= parts(tmp_quat_1);
                    tmp_quat2_1 = [q_a,q_b,q_c,q_d];
                    tmp_qVec2_1 = quaternion( tmp_quat2_1 );
                    tmp_R_i_iI = rotmat(tmp_qVec2_1,'frame');

                    tmp_quat_2 = quaternion(R2,"rotmat","frame");
                    [q_a,q_b,q_c,q_d]= parts(tmp_quat_2);
                    tmp_quat2_2 = [q_a,q_b,q_c,q_d];
                    tmp_qVec2_2 = quaternion( tmp_quat2_2 );
                    tmp_R_i_iI_2 = rotmat(tmp_qVec2_2,'frame');

                    fprintf("j=%d | [%7.6f %7.6f %7.6f %7.6f]\n", i, tmp_quat2_2(1), tmp_quat2_2(2), tmp_quat2_2(3), tmp_quat2_2(4));
                end

                %the inertial matrix centered at the origin of the i-frame(Sigma_i), in the i-frame(Sigma_i)
                I_i_i   = I_i_Ic + mi*cross_x( p_i_ic )'*cross_x( p_i_ic );

                phi_i_cad = [mi, mi*p_i_ic', I_i_i(1,1), I_i_i(2,2), I_i_i(3,3),  I_i_i(2,3), I_i_i(1,3), I_i_i(1,2) ]';
                phi = [phi; phi_i_cad];

                obj.data.link(i).m_i = mi;
                obj.data.link(i).p_i_ic = p_i_ic;
                obj.data.link(i).I_i_Ic = I_i_Ic;
            end
            obj.data.phi = phi;
        end

        function obj = set_root( obj )
            
            obj.data.root.p_j_ji   = [0 0 0]';
            obj.data.root.w_i_i    = [0 0 0]';
            obj.data.root.dw_i_i   = [0 0 0]';
            obj.data.root.ddp_i_0i = [0 0 obj.g]';
            obj.data.root.R_j_i    = eye(3);
            obj.data.root.selector = [0 0 1]';
            
        end

        function [result, result2, J_eff_i ] = calc_fk( obj, tmp_q)
            enable_endeff_site = 1;
            result = zeros(obj.num_of_links, 3);
            result2 = zeros(obj.num_of_links, 9);
            
            start_idx = 1;
            end_idx = 6;
            if obj.num_of_links ~= obj.dof
                start_idx = 2;
                end_idx = end_idx+1;
            end

            %% compute for L0
            if start_idx==2
                obj.data.link(1).R_0_i  = obj.data.root.R_j_i;
                obj.data.link(1).p_0_0i = obj.data.root.p_j_ji;
                result(1,:) = obj.data.link(1).p_0_0i';
            end

            %% compute for L1 - L6
            for k=start_idx:end_idx
                
                if k==1
                    %obj.data.link(k).R_0_i  = obj.data.link(1).R_j_i;
                    %obj.data.link(k).p_0_0i = obj.data.root.p_j_ji + obj.data.link(1).p_j_ji;
                    obj.data.link(k).R_0_i  = obj.data.root.R_j_i;
                    obj.data.link(k).p_0_0i = obj.data.root.p_j_ji;
                else
                    obj.data.link(k).q_i  = tmp_q( k-(start_idx-1), 1);
                    obj.data.link(k).R_j_i = obj.data.link(k).R_base_j_i * rotz( obj.data.link(k).q_i*180/pi );
                    obj.data.link(k).R_0_i  = obj.data.link(k-1).R_0_i * obj.data.link(k).R_j_i;
                    obj.data.link(k).p_0_0i = obj.data.link(k-1).p_0_0i + obj.data.link(k-1).R_0_i*obj.data.link(k).p_j_ji;
                end
                
                result(k,:) = obj.data.link(k).p_0_0i';
                result2(k,:) = reshape( obj.data.link(k).R_0_i, 1, 9);
            end

            %% consider eff site
            if enable_endeff_site
                %result = [result; result(end_idx,:) + (obj.data.link(end_idx).R_0_i*[0; 0; 0.011])' ]; %the edge eff
                result = [result; result(end_idx,:) + (obj.data.link(end_idx).R_0_i*[0; 0; 0.061])' ]; %the center of the eff mass
                result2 = [result2; result2(end_idx,:)];
            end

            %calc jacobian
            J_eff_i = zeros(6,6);
            for k=start_idx:end_idx
                R_i_0 = obj.data.link(k).R_0_i';
                R_0_eff = obj.data.link(end).R_0_i;
                R_i_eff = R_i_0 * R_0_eff;

                p_0_0eff = obj.data.link(end).p_0_0i;
                p_0_0i = obj.data.link(k).p_0_0i;
                p_i_ieff = R_i_0 * ( p_0_0eff - p_0_0i );
                
                tmp_J_eff_i = calc_Jacobian_i_j( R_i_eff, p_i_ieff);
                J_eff_i(:,k-1) = tmp_J_eff_i(:,6);
                
            end
        end

        function Y_full = calc_regressor( obj, tmp_q, tmp_dq, tmp_ddq )
            
            %2025.01.25
            % インデックスおかしいかも、、、
            %{
            %==========
            start_idx = 1;
            end_idx = 6;
            if obj.num_of_links ~= obj.dof
                start_idx = 2;
                end_idx = end_idx+1;
            end
            %% compute for L0
            if start_idx==2
                obj.data.link(1).R_j_i = obj.data.link(1).R_base_j_i * rotz( 0 );
                tmp_w_j_j  = obj.data.root.w_i_i;
                tmp_dw_j_j = obj.data.root.dw_i_i;   
                tmp_ddp_j_0j = obj.data.root.ddp_i_0i;

                % calc states
                tmp_w_i_i    = calc_w_i_i  ( obj.data.link(1).R_j_i, tmp_w_j_j, obj.data.link(1).selector, 0 );
                tmp_dw_i_i   = calc_dw_i_i ( obj.data.link(1).R_j_i, tmp_w_j_j, obj.data.link(1).selector, 0, tmp_dw_j_j, 0  );
        
                tmp_ddp_i_0i = calc_ddp_i_i( obj.data.link(1).R_j_i, tmp_w_j_j, tmp_dw_j_j, tmp_ddp_j_0j, obj.data.link(1).p_j_ji );
                tmp_J_i_j    = calc_Jacobian_i_j(obj.data.link(1).R_j_i, obj.data.link(1).p_j_ji);
                tmp_J_j_i    = tmp_J_i_j';

                % set states
                obj.data.link(1).w_i_i    = tmp_w_i_i;
                obj.data.link(1).dw_i_i   = tmp_dw_i_i;
                obj.data.link(1).ddp_i_0i = tmp_ddp_i_0i;
                obj.data.link(1).J_j_i    = tmp_J_j_i;

                obj.data.link(1).Y_i_i    = calc_body_Y_i_i( obj.data.link(1).ddp_i_0i, obj.data.link(1).w_i_i, obj.data.link(1).dw_i_i );
                

            end
            %}
            
            % calc link regressor, Jacobian
            %% compute for L1 - L6
            %for k=start_idx:end_idx
            %============
            for k=1:obj.dof
                % read current states
                obj.data.link(k).q_i  = tmp_q(k,1);
                obj.data.link(k).dq_i = tmp_dq(k,1);
                obj.data.link(k).ddq_i = tmp_ddq(k,1);  
                
                obj.data.link(k).R_j_i = obj.data.link(k).R_base_j_i * rotz( obj.data.link(k).q_i*180/pi );
        
                
                if k==1
                    tmp_w_j_j  = obj.data.root.w_i_i;
                    tmp_dw_j_j = obj.data.root.dw_i_i;   
                    tmp_ddp_j_0j = obj.data.root.ddp_i_0i;
                else
                    tmp_w_j_j    = obj.data.link(k-1).w_i_i;
                    tmp_dw_j_j   = obj.data.link(k-1).dw_i_i;
                    tmp_ddp_j_0j = obj.data.link(k-1).ddp_i_0i;
                end

                
                % calc states
                tmp_w_i_i    = calc_w_i_i  ( obj.data.link(k).R_j_i, tmp_w_j_j, obj.data.link(k).selector, obj.data.link(k).dq_i );
                tmp_dw_i_i   = calc_dw_i_i ( obj.data.link(k).R_j_i, tmp_w_j_j, obj.data.link(k).selector, obj.data.link(k).dq_i, tmp_dw_j_j, obj.data.link(k).ddq_i  );
        
                tmp_ddp_i_0i = calc_ddp_i_i( obj.data.link(k).R_j_i, tmp_w_j_j, tmp_dw_j_j, tmp_ddp_j_0j, obj.data.link(k).p_j_ji );
                tmp_J_i_j    = calc_Jacobian_i_j(obj.data.link(k).R_j_i, obj.data.link(k).p_j_ji);
                tmp_J_j_i    = tmp_J_i_j';

                % set states
                obj.data.link(k).w_i_i    = tmp_w_i_i;
                obj.data.link(k).dw_i_i   = tmp_dw_i_i;
                obj.data.link(k).ddp_i_0i = tmp_ddp_i_0i;
                obj.data.link(k).J_j_i    = tmp_J_j_i;

                obj.data.link(k).Y_i_i    = calc_body_Y_i_i( obj.data.link(k).ddp_i_0i, obj.data.link(k).w_i_i, obj.data.link(k).dw_i_i );
                
            end
            
            Y_tmp = zeros( 6*obj.dof, 10*obj.dof );
            for k = 1:obj.dof
                
                Y_tmp( (1:6)+(k-1)*6, (1:10)+(k-1)*10  ) = obj.data.link(k).Y_i_i;
                
                tmp_J_2_i = eye(6);
                for k2 = 1:(k-1)
                    tmp_idx = (k-1)-(k2)+2;
                    tmp_J_2_i = obj.data.link( tmp_idx ).J_j_i*tmp_J_2_i;
                    Y_tmp( (1:6)+(tmp_idx-2)*6, (1:10)+(k-1)*10  ) = tmp_J_2_i * obj.data.link(k).Y_i_i; 
                end
            end

            Y_full = zeros( obj.dof, size(Y_tmp,2) );
            for k=1:obj.dof
                tmp_idx = (6*(k-1)+1):(6*(k));
                select_v = [ zeros(1,3), obj.data.link(k).selector' ];
                Y_full(k,:) = Y_tmp( tmp_idx( select_v==1 ),: );
            end

        end


        function Y_base_full = calc_base_regressor( obj, tmp_q, tmp_dq, tmp_ddq )
            
            % calc link regressor, Jacobian
            for k=1:obj.dof
                % read current states
                obj.data.link(k).q_i  = tmp_q(k,1);
                obj.data.link(k).dq_i = tmp_dq(k,1);
                obj.data.link(k).ddq_i = tmp_ddq(k,1);  
                
                obj.data.link(k).R_j_i = obj.data.link(k).R_base_j_i * rotz( obj.data.link(k).q_i*180/pi );
        
                
                if k==1
                    tmp_w_j_j  = obj.data.root.w_i_i;
                    tmp_dw_j_j = obj.data.root.dw_i_i;   
                    tmp_ddp_j_0j = obj.data.root.ddp_i_0i;
                else
                    tmp_w_j_j    = obj.data.link(k-1).w_i_i;
                    tmp_dw_j_j   = obj.data.link(k-1).dw_i_i;
                    tmp_ddp_j_0j = obj.data.link(k-1).ddp_i_0i;
                end

                
                % calc states
                tmp_w_i_i    = calc_w_i_i  ( obj.data.link(k).R_j_i, tmp_w_j_j, obj.data.link(k).selector, obj.data.link(k).dq_i );
                tmp_dw_i_i   = calc_dw_i_i ( obj.data.link(k).R_j_i, tmp_w_j_j, obj.data.link(k).selector, obj.data.link(k).dq_i, tmp_dw_j_j, obj.data.link(k).ddq_i  );
        
                tmp_ddp_i_0i = calc_ddp_i_i( obj.data.link(k).R_j_i, tmp_w_j_j, tmp_dw_j_j, tmp_ddp_j_0j, obj.data.link(k).p_j_ji );
                tmp_J_i_j    = calc_Jacobian_i_j(obj.data.link(k).R_j_i, obj.data.link(k).p_j_ji);
                tmp_J_j_i    = tmp_J_i_j';

                % set states
                obj.data.link(k).w_i_i    = tmp_w_i_i;
                obj.data.link(k).dw_i_i   = tmp_dw_i_i;
                obj.data.link(k).ddp_i_0i = tmp_ddp_i_0i;
                obj.data.link(k).J_j_i    = tmp_J_j_i;

                obj.data.link(k).Y_i_i    = calc_body_Y_i_i( obj.data.link(k).ddp_i_0i, obj.data.link(k).w_i_i, obj.data.link(k).dw_i_i );
                
            end
            
            Y_tmp = zeros( 6*obj.dof, 10*obj.dof );
            for k = 1:obj.dof
                
                Y_tmp( (1:6)+(k-1)*6, (1:10)+(k-1)*10  ) = obj.data.link(k).Y_i_i;
                
                tmp_J_2_i = eye(6);
                for k2 = 1:(k-1)
                    tmp_idx = (k-1)-(k2)+2;
                    tmp_J_2_i = obj.data.link( tmp_idx ).J_j_i*tmp_J_2_i;
                    Y_tmp( (1:6)+(tmp_idx-2)*6, (1:10)+(k-1)*10  ) = tmp_J_2_i * obj.data.link(k).Y_i_i; 
                end
            end

            % calc base states
            tmp_w_j_j  = obj.data.root.w_i_i;
            tmp_dw_j_j = obj.data.root.dw_i_i;   
            tmp_ddp_j_0j = obj.data.root.ddp_i_0i;

            tmp_w_i_i    = calc_w_i_i  ( eye(3), tmp_w_j_j, [0;0;0], 0 );
            tmp_dw_i_i   = calc_dw_i_i ( eye(3), tmp_w_j_j, [0;0;0], 0, tmp_dw_j_j, 0  );
            tmp_ddp_i_0i = calc_ddp_i_i( eye(3), tmp_w_j_j, tmp_dw_j_j, tmp_ddp_j_0j, [0;0;0] );

            tmp_J_i_j    = calc_Jacobian_i_j( eye(3), [0;0;0]);
            tmp_J_j_i    = tmp_J_i_j';
            
            Y_0_0 = calc_body_Y_i_i( tmp_ddp_i_0i, tmp_w_i_i, tmp_dw_i_i );
            
            Y_base_full = Y_0_0;
            for i=1:obj.dof
                tmp_idx = [(i-1)*10+1:i*10];
                Y_base_full = [Y_base_full, obj.data.link(1).J_j_i * Y_tmp(1:6, tmp_idx )];
            end
            

        end
        
        function J_eff = calc_eff_jacobian( obj, tmp_q)
            
            %link(0).R_base_j_i = [1     0     0     0     1     0     0     0     1];
            link(1).R_base_j_i = [1     0     0     0     1     0     0     0     1];
            link(2).R_base_j_i = [1.0000         0         0         0    0.0000   -1.0000         0    1.0000    0.0000];
            link(3).R_base_j_i = [1     0     0     0     1     0     0     0     1];
            link(4).R_base_j_i = [0.0000         0   -1.0000         0    1.0000         0    1.0000         0    0.0000];
            link(5).R_base_j_i = [ 0.0000         0    1.0000         0    1.0000         0   -1.0000         0    0.0000];
            link(6).R_base_j_i = eye(3);

            %link(1).p_j_ji = [0     0     0]';
            link(1).p_j_ji = [0         0    0.0910]';
            link(2).p_j_ji = [0         0    0.0445]';
            link(3).p_j_ji = [0   -0.1350         0]';
            link(4).p_j_ji = [ 0.0615   -0.0380         0]';
            link(5).p_j_ji = [0    0.000004    0.0584]';
            link(6).p_j_ji = [0;0;0];

            for k=1:6
                if k==1
                
                    link(k).q_i  = tmp_q( k, 1);
                    link(k).R_j_i =  eye(3) * rotz( link(k).q_i*180/pi );
                    link(k).R_0_i  = eye(3) * link(k).R_j_i;
                    link(k).p_0_0i = [0;0;0];
                else
                
                    link(k).q_i  = tmp_q( k, 1);
                    link(k).R_j_i = reshape(link(k).R_base_j_i,3,3) * rotz( link(k).q_i*180/pi );
                    link(k).R_0_i  = link(k-1).R_0_i * link(k).R_j_i;
                    link(k).p_0_0i = link(k-1).p_0_0i + link(k-1).R_0_i*link(k).p_j_ji;

                   %link(k).p_0_0i = link(k-1).p_0_0i + link(k-1).R_0_i*link(k).p_j_ji;
                end
            end

            J_eff = zeros(6,6);
            for k=1:6
                R_i_0 = link(k).R_0_i';
                R_0_eff = link(end).R_0_i;
                R_i_eff = R_i_0 * R_0_eff;

                p_0_0eff = link(end).p_0_0i;
                p_0_0i = link(k).p_0_0i;
                p_i_ieff = R_i_0 * ( p_0_0eff - p_0_0i );
                
                tmp_J_eff_i = calc_Jacobian_i_j( R_i_eff, p_i_ieff);
                J_eff(:,k) = tmp_J_eff_i(:,6);
            end

            enable_endeff_site = 1;
            
            start_idx = 1;
            end_idx = 6;
            if obj.num_of_links ~= obj.dof
                start_idx = 2;
                end_idx = end_idx+1;
            end

            %% compute for L0
            if start_idx==2
                obj.data.link(1).R_0_i  = obj.data.root.R_j_i;
                obj.data.link(1).p_0_0i = obj.data.root.p_j_ji;
                result(1,:) = obj.data.link(1).p_0_0i';
            end

            %% compute for L1 - L6
            for k=start_idx:end_idx
                
                if k==1
                    %obj.data.link(k).R_0_i  = obj.data.link(1).R_j_i;
                    %obj.data.link(k).p_0_0i = obj.data.root.p_j_ji + obj.data.link(1).p_j_ji;
                    obj.data.link(k).R_0_i  = obj.data.root.R_j_i;
                    obj.data.link(k).p_0_0i = obj.data.root.p_j_ji;
                else
                    obj.data.link(k).q_i  = tmp_q( k-(start_idx-1), 1);
                    obj.data.link(k).R_j_i = obj.data.link(k).R_base_j_i * rotz( obj.data.link(k).q_i*180/pi );
                    obj.data.link(k).R_0_i  = obj.data.link(k-1).R_0_i * obj.data.link(k).R_j_i;
                    obj.data.link(k).p_0_0i = obj.data.link(k-1).p_0_0i + obj.data.link(k-1).R_0_i*obj.data.link(k).p_j_ji;
                end
                
                result(k,:) = obj.data.link(k).p_0_0i';
                result2(k,:) = reshape( obj.data.link(k).R_0_i, 1, 9);
            end
            %{
            %calc jacobian
            J_eff = zeros(6,6);
            for k=start_idx:end_idx
                R_i_0 = obj.data.link(k).R_0_i';
                R_0_eff = obj.data.link(end).R_0_i;
                R_i_eff = R_i_0 * R_0_eff;

                p_0_0eff = obj.data.link(end).p_0_0i;
                p_0_0i = obj.data.link(k).p_0_0i;
                p_i_ieff = R_i_0 * ( p_0_0eff - p_0_0i );
                
                tmp_J_eff_i = calc_Jacobian_i_j( R_i_eff, p_i_ieff);
                J_eff(:,k-1) = tmp_J_eff_i(:,6);
                
            end
            %}
        end
        function [J_eff ] = calc_eff_jacobian2( obj, tmp_q)
            
            local_data(1).R_base_j_i = [1     0     0     0     1     0     0     0     1];
            local_data(2).R_base_j_i = [1     0     0     0     1     0     0     0     1];
            local_data(3).R_base_j_i = [1.0000         0         0         0    0.0000   -1.0000         0    1.0000    0.0000];
            local_data(4).R_base_j_i = [1     0     0     0     1     0     0     0     1];
            local_data(5).R_base_j_i = [0.0000         0   -1.0000         0    1.0000         0    1.0000         0    0.0000];
            local_data(6).R_base_j_i = [ 0.0000         0    1.0000         0    1.0000         0   -1.0000         0    0.0000];
            local_data(7).R_base_j_i = [ 0.0000         0   -1.0000         0    1.0000         0    1.0000         0    0.0000];

            local_data(1).p_j_ji = [0     0     0]';
            local_data(2).p_j_ji = [0         0    0.0910]';
            local_data(3).p_j_ji = [0         0    0.0445]';
            local_data(4).p_j_ji = [0   -0.1350         0]';
            local_data(5).p_j_ji = [ 0.0615   -0.0380         0]';
            local_data(6).p_j_ji = [0    0.000004    0.0584]';
            local_data(7).p_j_ji = [0.0590, 0, 0]';

            start_idx = 2;
            end_idx = 7;
            
            %% compute for L0
            if start_idx==2
                link(1).R_0_i  = eye(3);
                link(1).p_0_0i = zeros(3,1);
            end

            %% compute for L1 - L6
            for k=start_idx:end_idx
                
                if k==1
                    link(k).R_0_i  = eye(3);
                    link(k).p_0_0i = zeros(3,1);
                else
                    link(k).q_i  = tmp_q( k-(start_idx-1), 1);
                    link(k).R_j_i = reshape(local_data(k).R_base_j_i,3,3) * rotz( link(k).q_i*180/pi );
                    link(k).R_0_i  = link(k-1).R_0_i * link(k).R_j_i;
                    link(k).p_0_0i = link(k-1).p_0_0i + link(k-1).R_0_i*local_data(k).p_j_ji;
                end
                
            end

            %calc jacobian
            J_eff = zeros(6,6);
            for k=start_idx:end_idx
                R_i_0 = link(k).R_0_i';
                R_0_eff = link(end).R_0_i;
                R_i_eff = R_i_0 * R_0_eff;

                p_0_0eff = link(end).p_0_0i;
                p_0_0i = link(k).p_0_0i;
                p_i_ieff = R_i_0 * ( p_0_0eff - p_0_0i );
                
                tmp_J_eff_i = calc_Jacobian_i_j( R_i_eff, p_i_ieff);
                J_eff(:,k-1) = tmp_J_eff_i(:,6);
                
            end
        end

        %{
        function J_base_eff = calc_eff_jacobian( obj, tmp_q )
            local_data(1).R_base_j_i = [1     0     0     0     1     0     0     0     1];
            local_data(2).R_base_j_i = [1     0     0     0     1     0     0     0     1];
            local_data(3).R_base_j_i = [1.0000         0         0         0    0.0000   -1.0000         0    1.0000    0.0000];
            local_data(4).R_base_j_i = [1     0     0     0     1     0     0     0     1];
            local_data(5).R_base_j_i = [0.0000         0   -1.0000         0    1.0000         0    1.0000         0    0.0000];
            local_data(6).R_base_j_i = [ 0.0000         0    1.0000         0    1.0000         0   -1.0000         0    0.0000];
            % calc link regressor, Jacobian
            for k=1:obj.dof
                % read current states
                obj.data.link(k).q_i  = tmp_q(k,1);
                obj.data.link(k).R_j_i = obj.data.link(k).R_base_j_i * rotz( obj.data.link(k).q_i*180/pi );
                
                % calc states
                tmp_J_i_j    = calc_Jacobian_i_j(obj.data.link(k).R_j_i, obj.data.link(k).p_j_ji);
                tmp_J_j_i    = tmp_J_i_j';
                obj.data.link(k).J_j_i    = tmp_J_j_i;
            end
            J_base_eff = eye(6);
            for k=1:obj.dof
                J_base_eff = J_base_eff*obj.data.link(k).J_j_i;
            end
        end

        function Y_base_full = calc_eff_jacobian2( obj, tmp_q, tmp_dq, tmp_ddq )
            
            local_data(1).R_base_j_i = [1     0     0     0     1     0     0     0     1];
            local_data(2).R_base_j_i = [1     0     0     0     1     0     0     0     1];
            local_data(3).R_base_j_i = [1.0000         0         0         0    0.0000   -1.0000         0    1.0000    0.0000];
            local_data(4).R_base_j_i = [1     0     0     0     1     0     0     0     1];
            local_data(5).R_base_j_i = [0.0000         0   -1.0000         0    1.0000         0    1.0000         0    0.0000];
            local_data(6).R_base_j_i = [ 0.0000         0    1.0000         0    1.0000         0   -1.0000         0    0.0000];

            local_data(1).p_j_ji = [0     0     0]';
            local_data(2).p_j_ji = [0         0    0.0910]';
            local_data(3).p_j_ji = [0         0    0.0445]';
            local_data(4).p_j_ji = [0   -0.1350         0]';
            local_data(5).p_j_ji = [ 0.0615   -0.0380         0]';
            local_data(6).p_j_ji = [0    0.000004    0.0584]';

            % calc link regressor, Jacobian
            for k=1:obj.dof
                % read current states
                obj.data.link(k).q_i  = tmp_q(k,1);
                obj.data.link(k).dq_i = tmp_dq(k,1);
                obj.data.link(k).ddq_i = tmp_ddq(k,1);  
                R_j_i = reshape(local_data(k).R_base_j_i,3,3) * rotz( tmp_q(k,1)*180/pi );
                
                if k==1
                    tmp_w_j_j  = obj.data.root.w_i_i;
                    tmp_dw_j_j = obj.data.root.dw_i_i;   
                    tmp_ddp_j_0j = obj.data.root.ddp_i_0i;
                else
                    tmp_w_j_j    = obj.data.link(k-1).w_i_i;
                    tmp_dw_j_j   = obj.data.link(k-1).dw_i_i;
                    tmp_ddp_j_0j = obj.data.link(k-1).ddp_i_0i;
                end

                % calc states
                tmp_w_i_i    = calc_w_i_i  ( obj.data.link(k).R_j_i, tmp_w_j_j, obj.data.link(k).selector, obj.data.link(k).dq_i );
                tmp_dw_i_i   = calc_dw_i_i ( obj.data.link(k).R_j_i, tmp_w_j_j, obj.data.link(k).selector, obj.data.link(k).dq_i, tmp_dw_j_j, obj.data.link(k).ddq_i  );
        
                tmp_ddp_i_0i = calc_ddp_i_i( obj.data.link(k).R_j_i, tmp_w_j_j, tmp_dw_j_j, tmp_ddp_j_0j, obj.data.link(k).p_j_ji );
                tmp_J_i_j    = calc_Jacobian_i_j(R_j_i, local_data(k).p_j_ji);

                tmp_J_j_i    = tmp_J_i_j';
                
                % set states
                obj.data.link(k).w_i_i    = tmp_w_i_i;
                obj.data.link(k).dw_i_i   = tmp_dw_i_i;
                obj.data.link(k).ddp_i_0i = tmp_ddp_i_0i;
                obj.data.link(k).J_j_i    = tmp_J_j_i;

                local_data(k).J_i_j    = tmp_J_i_j;


                obj.data.link(k).Y_i_i    = calc_body_Y_i_i( obj.data.link(k).ddp_i_0i, obj.data.link(k).w_i_i, obj.data.link(k).dw_i_i );
                

            end
            J_base_eff = eye(6);
            for k=1:obj.dof
                J_base_eff = J_base_eff*local_data(k).J_i_j;
                %J_base_eff = J_base_eff*reshape(local_data(k).R_base_j_i,3,3);
            end
            
            Y_tmp = zeros( 6*obj.dof, 10*obj.dof );
            for k = 1:obj.dof
                
                Y_tmp( (1:6)+(k-1)*6, (1:10)+(k-1)*10  ) = obj.data.link(k).Y_i_i;
                
                tmp_J_2_i = eye(6);
                for k2 = 1:(k-1)
                    tmp_idx = (k-1)-(k2)+2;
                    tmp_J_2_i = obj.data.link( tmp_idx ).J_j_i*tmp_J_2_i;
                    Y_tmp( (1:6)+(tmp_idx-2)*6, (1:10)+(k-1)*10  ) = tmp_J_2_i * obj.data.link(k).Y_i_i; 
                end
            end

            % calc base states
            tmp_w_j_j  = obj.data.root.w_i_i;
            tmp_dw_j_j = obj.data.root.dw_i_i;   
            tmp_ddp_j_0j = obj.data.root.ddp_i_0i;

            tmp_w_i_i    = calc_w_i_i  ( eye(3), tmp_w_j_j, [0;0;0], 0 );
            tmp_dw_i_i   = calc_dw_i_i ( eye(3), tmp_w_j_j, [0;0;0], 0, tmp_dw_j_j, 0  );
            tmp_ddp_i_0i = calc_ddp_i_i( eye(3), tmp_w_j_j, tmp_dw_j_j, tmp_ddp_j_0j, [0;0;0] );

            tmp_J_i_j    = calc_Jacobian_i_j( eye(3), [0;0;0]);
            tmp_J_j_i    = tmp_J_i_j';
            
            Y_0_0 = calc_body_Y_i_i( tmp_ddp_i_0i, tmp_w_i_i, tmp_dw_i_i );
            
            Y_base_full = Y_0_0;
            for i=1:obj.dof
                tmp_idx = [(i-1)*10+1:i*10];
                Y_base_full = [Y_base_full, obj.data.link(1).J_j_i * Y_tmp(1:6, tmp_idx )];
            end

        end
        %}


        function obj = set_compose_X_Z_phib(obj, Y_all)
            
            ib = calc_ib(Y_all);
            obj.data.X = conv_ib2X(ib, 10*obj.dof);
            obj.data.Z = calc_Z(obj.data.X, Y_all);
            obj.data.phib = obj.data.Z*obj.data.phi;
            obj.data.ib = ib;
            
        end
    end
end