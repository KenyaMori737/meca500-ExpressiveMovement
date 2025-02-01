function J_i_j = calc_Jacobian_i_j( R_j_i, p_j_ji )
    
    R_i_j = R_j_i';
    J_i_j = [R_i_j, -R_i_j*cross_x(p_j_ji); 
             zeros(3,3),     R_i_j];

end