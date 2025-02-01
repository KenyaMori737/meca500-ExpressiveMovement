classdef mecademic_meca500_sharedData < handle
%% 2024 05 26 new
    properties (Access = public)
      cad;
      link;
      phi;
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
      tau_limit_adaptive;
      ang_limit; %rad
      vel_limit; %rad/s
      acc_limit; %rad/s^2
      CoP_limit;
      
      last_coef;
      ib;
      Yb0;
      Yb0T_Yb0;
      F0;
      rsd_p_ident;
    end
end