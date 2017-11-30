function [c,ceq,Jc,Jceq] = circleCon(z,sysParam)
    
    N = sysParam.N; % Time step for free time optimization (number of imputs)
    delta_ts = sysParam.delta_ts;
    isTargetConstraint = sysParam.isTargetConstraint; % 1 for target as constraint
    zDim_per_step = sysParam.zDim_per_step; % 1 for tau, 2 for state, 1 for b

    zDim = sysParam.zDim; % Dimension of states: N for input and 2*(N+1) for states

    StartPos = sysParam.StartPos;
    TargetPos = sysParam.TargetPos;

    u_lb = sysParam.u_lb;
    u_ub = sysParam.u_ub;

    track_lb = sysParam.track_lb;
    track_rb = sysParam.track_rb;

    %% Inequality constraint: 
    numCon_b = 1; % 1 for b,
    numCon_u = 2*N; % 2*N for input (upper and lower limits)
    numCon_tau = 0; % (N+1) for tau constraints (already considered in the equality constraint)
    numCon_circ = 2*(N+1); % 2*(N+1) for circle constraints,
    c = zeros(numCon_b + numCon_u + numCon_tau + numCon_circ, 1);
    Jc = zeros(numCon_b + numCon_u + numCon_tau + numCon_circ, zDim)'; % Matlab uses transpose of Jacobian
    c_circ = size(numCon_circ,1);
    Jc_circ = zeros(numCon_circ,zDim);
    for k = 0:N
        c_circ(2*k+1:2*k+2,1) = [z(N+zDim_per_step*k+2,1).^2 + z(N+zDim_per_step*k+3,1).^2 - track_lb.^2;...
                                 - z(N+zDim_per_step*k+2,1).^2 - z(N+zDim_per_step*k+3,1).^2 + track_rb.^2]; 
        Jc_circ(2*k+1,N+zDim_per_step*k+2) = 2*z(N+zDim_per_step*k+2,1);
        Jc_circ(2*k+1,N+zDim_per_step*k+3) = 2*z(N+zDim_per_step*k+3,1);
        Jc_circ(2*k+2,N+zDim_per_step*k+2) = -2*z(N+zDim_per_step*k+2,1);
        Jc_circ(2*k+2,N+zDim_per_step*k+3) = -2*z(N+zDim_per_step*k+3,1);
    end
    c = [-z(zDim,1)+0*10; % b need to be positive or obove a value
         z(1:N,1)-u_ub.*ones(N,1);... % control input upper limit
         -z(1:N,1)+u_lb.*ones(N,1);... % control input lower limit
         c_circ];
    Jc = [[zeros(1,zDim-1) -1];...
          [eye(N) zeros(N,zDim-N)];...
          [-eye(N) zeros(N,zDim-N)];...
          Jc_circ]'; % Matlab uses transpose of Jacobian
    
    %% Analytically calculate the Jacobian of f so it can be used  
    % [x_dot;y_dot]=[0 0.310625 0; 0 0 1]*[x;y;u]  
    
    
    b = z(zDim,1);
    %% Equality constraint
    if isTargetConstraint == 0 % Target as a part of the cost function, so it does not apper in the constraint
        for k = 0:N-1 % Euler Forward
            ceq(zDim_per_step*k+1:zDim_per_step*k+zDim_per_step,1) = [z(N+zDim_per_step*(k+1)+1 : N+zDim_per_step*(k+1)+zDim_per_step , 1)]...
                                                                     -[z(N+zDim_per_step*k+1 : N+zDim_per_step*k+zDim_per_step , 1)]...
                                                                     -delta_ts*[b;b*[0 0.310625 0;0 0 1]*[z(N+zDim_per_step*k+2,1);z(N+zDim_per_step*k+3,1);z(k+1,1)];0];
            
%             Jceq(zDim_per_step*k+1:zDim_per_step*k+zDim_per_step,:) = [];
        end
        ceq = [ceq;z(N+2:N+3,1)-StartPos]; % Initial condition ?????????? Is is necessary?
        Jceq=[];
%         Jceq = [Jceq;...
%                zeros(1,N+1) 1 zeros(1,zDim-N-2);...
%                zeros(1,N+2) 1 zeros(1,zDim-N-3)]'; % Matlab uses transpose of Jacobian
    elseif isTargetConstraint == 1 % Target as constraint
        for k = 0:N-1 % Euler Forward
            ceq(zDim_per_step*k+1:zDim_per_step*k+zDim_per_step,1) = [z(N+zDim_per_step*(k+1)+1 : N+zDim_per_step*(k+1)+zDim_per_step , 1)]...
                                                                     -[z(N+zDim_per_step*k+1 : N+zDim_per_step*k+zDim_per_step , 1)]...
                                                                     -delta_ts*[b;b*[0 0.310625 0;0 0 1]*[z(N+zDim_per_step*k+2,1);z(N+zDim_per_step*k+3,1);z(k+1,1)];0];
            
%             Jceq(zDim_per_step*k+1:zDim_per_step*k+zDim_per_step,:) = [];
        end
        ceq = [ceq;z(zDim-2:zDim-1,1)-TargetPos;z(N+2:N+3,1)-StartPos]; % Initial condition ?????????? Is is necessary?
        Jceq=[];
%         Jceq = [Jceq;...
%                zeros(1,zDim-3) 1 0 0;...
%                zeros(1,zDim-2) 1 0;... % Matlab uses transpose of Jacobian   
%                zeros(1,N+1) 1 zeros(1,zDim-N-2);...
%                zeros(1,N+2) 1 zeros(1,zDim-N-3)]';    
    end
           
end