%% 2024.05.26 Author@Kenya Mori
%% 2024.07.11 Considered L0 link and eff site
%% compile command: codegen calc_fk_mecademic_for_codegen -args {[0 0 0 0 0 0]}

function fk_result = calc_fk_mecademic_for_codegen( tmp_q )

        
        dof = 6;
        num_of_links = 7;
        tmp_link = struct("p_j_ji", [0.0 0.0 0.0]', ...
                          "selector", [0.0 0.0 1.0]', ...
                          "R_base_j_i", eye(3), ...
                          "q_i", 0, ...
                          "dq_i", 0, ...
                          "ddq_i", 0, ...
                          "R_j_i", eye(3), ...
                          "w_i_i", [0.0 0.0 0.0]', ...
                          "dw_i_i", [0.0 0.0 0.0]', ...
                          "ddp_i_0i", [0.0 0.0 0.0]', ...
                          "J_j_i", eye(6), ...
                          "Y_i_i", zeros(6,10), ...
                          "R_0_i", eye(3), ...
                          "p_0_0i", [0.0 0.0 0.0]' );
        
        link = [tmp_link, tmp_link, tmp_link, tmp_link, tmp_link, tmp_link, tmp_link];

        param = [ 0          0          0       0   0   1   0   0  0    1        0       0      0; ...
                  0	         0	        0.091	0	0	1	0   0   0	1	     0	     0	    0; ...
                  0	         0	        0.0445	0	0	1	0   0   0	0.7071	-0.7071	 0	    0; ...
                  0	        -0.135	    0	    0	0	1	0   0   0	1	     0	     0	    0; ...
                  0.0615	-0.038	    0	    0	0	1	0   0   0	0.7071	 0	     0.7071	0; ...
                  0	         0.000004	0.0584	0	0	1	0   0   0	0.7071	 0	    -0.7071	0; ...
                  0.059	     0	        0	    0	0	1	0   0   0	0.7071	 0	     0.7071	0];

        fk_result = zeros( num_of_links+1, 3);

        start_idx = 1;
        end_idx = 6;
        if num_of_links ~= dof
            start_idx = 2;
            end_idx = end_idx+1;
        end

        if start_idx==2
            link(1).R_0_i  = eye(3);
            link(1).p_0_0i = [0;0;0];
            fk_result(1,:) = link(1).p_0_0i';
        end

        %for k=1:dof
        for k=start_idx:end_idx
            
            link(k).p_j_ji = [param(k,1), param(k,2), param(k,3)]';
            link(k).selector = [param(k,4), param(k,5), param(k,6)]';
            %link(k).R_base_j_i = rotz( param(k,7) )*roty( param(k,8) )*rotz( param(k,9) );
            quat = param(k,10:13);
            qVec = quaternion( quat );
            link(k).R_base_j_i = rotmat(qVec,'frame')';

            % read current states
            link(k).q_i  = tmp_q( k-(start_idx-1) );
            link(k).R_j_i = link(k).R_base_j_i * rotz( link(k).q_i*180/pi );

            if k==1
                %link(k).R_0_i  = link(1).R_j_i;
                %link(k).p_0_0i = [0 0 0]' + link(1).p_j_ji;
                link(k).R_0_i  = eye(3);
                link(k).p_0_0i = [0 0 0]';
            else
                link(k).R_0_i  = link(k-1).R_0_i * link(k).R_j_i;
                link(k).p_0_0i = link(k-1).p_0_0i + link(k-1).R_0_i*link(k).p_j_ji;
            end
            
            fk_result(k,:) = link(k).p_0_0i';
        end
        %fk_result(dof+1,:) = fk_result(dof,:) + ( link(dof).R_0_i*[0,0,+0.044]' )';
        
        %% fk for eff
        %fk_result(end,:) = fk_result(end_idx,:) + (link(end_idx).R_0_i*[0; 0; 0.011])'; %the edge of eff
        fk_result(end,:) = fk_result(end_idx,:) + (link(end_idx).R_0_i*[0; 0; 0.061])'; %the center of the eff mass

        function [R] = rotz(ang)
            ang_rad = ang*pi/180;
            R = eye(3);
            R(1,1) = cos(ang_rad); R(1,2) = -sin(ang_rad); 
            R(2,1) = sin(ang_rad); R(2,2) =  cos(ang_rad);
    
        end
        function [R] = roty(ang)
            ang_rad = ang*pi/180;
            R = eye(3);
            R(1,1) = cos(ang_rad); R(1,3) =  sin(ang_rad); 
            R(3,1) =-sin(ang_rad); R(3,3) =  cos(ang_rad);
    
        end
end