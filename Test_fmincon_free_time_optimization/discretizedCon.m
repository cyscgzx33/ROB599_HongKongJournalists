function [c,ceq,Jc,Jceq] = discretizedCon(x,sysParam)
    
    delta_t=sysParam(1,1);
    N=sysParam(2,1);
    xDim=sysParam(3,1);
    track_bl=sysParam(4,1);
    track_br=sysParam(5,1);
    isTargetConstraint = sysParam(6,1); % 1 for target as constraint
    trackSegNum=sysParam(7,1);
    
    track_ang = linspace(pi,0,trackSegNum);
    track_bl = [track_bl*cos(track_ang);track_bl*sin(track_ang)];
    track_br = [track_br*cos(track_ang);track_br*sin(track_ang)];
    
    u_lb = -10;
    u_ub = 10;
    
    %% Inequality constraint
    c_circ = zeros(2*(N+1),1);
    Jc_circ = zeros(2*(N+1),xDim);
    for k = 0:N
        % For each point, find the nearest points on bl and br, and do
        % interpolation to find out the constraint
        distance2_bl=(track_bl(1,:)-x(1,1)).^2+(track_bl(2,:)-x(2,1)).^2;
        distance2_bl_sorted=sort(distance2_bl);
        [~,nearestIndex_bl]=intersect(distance2_bl,distance2_bl_sorted(1:2));
        v_bl=[track_bl(1,max(nearestIndex_bl))-track_bl(1,min(nearestIndex_bl));track_bl(2,max(nearestIndex_bl))-track_bl(2,min(nearestIndex_bl))];
        distance2_br=(track_br(1,:)-x(1,1)).^2+(track_br(2,:)-x(2,1)).^2;
        distance2_br_sorted=sort(distance2_br);
        [~,nearestIndex_br]=intersect(distance2_br,distance2_br_sorted(1:2));
        v_br=[track_br(1,max(nearestIndex_br))-track_br(1,min(nearestIndex_br));track_br(2,max(nearestIndex_br))-track_br(2,min(nearestIndex_br))];
        c_circ(2*k+1:2*k+2,1)=[v_bl(1)*(x(N+2*k+2,1)-track_bl(2,min(nearestIndex_bl)))-v_bl(2)*(x(N+2*k+1,1)-track_bl(1,min(nearestIndex_bl)));...
                               -v_br(1)*(x(N+2*k+2,1)-track_br(2,min(nearestIndex_br)))+v_br(2)*(x(N+2*k+1,1)-track_br(1,min(nearestIndex_br)))];
        Jc_circ(2*k+1,N+2*k+1) = -v_bl(2);
        Jc_circ(2*k+1,N+2*k+2) = v_bl(1);
        Jc_circ(2*k+2,N+2*k+1) = v_br(2);
        Jc_circ(2*k+2,N+2*k+2) = -v_br(1);
    end
    c = [x(1:N,1)-u_ub.*ones(size(x(1:N,1)));...
        -x(1:N,1)+u_lb.*ones(size(x(1:N,1)));...
        c_circ];
    Jc = [[eye(N) zeros(N,xDim-N)];...
          [-eye(N) zeros(N,xDim-N)];...
          Jc_circ]'; % Matlab uses transpose of Jacobian
    
    %% Equality constraint
    if isTargetConstraint == 0 
        for k = 0:N-1 % Euler Forward
            ceq(2*k+1,1) = x(N+2*(k+1)+1) - x(N+2*k+1) - delta_t*0.310625*x(N+2*k+2);
            ceq(2*k+2,1) = x(N+2*(k+1)+2) - x(N+2*k+2) - delta_t*1*x(k+1);
            Jceq(2*k+1,:) = [zeros(1,N+2*k) -1 -delta_t*0.310625 1 zeros(1,xDim-N-2*k-3)];
            Jceq(2*k+2,:) = [zeros(1,k) -delta_t zeros(1,N-k-1) zeros(1,2*k+1) -1 0 1 zeros(1,xDim-N-2*k-4)];
        end
        ceq = [ceq;x(N+1,1)+1; x(N+2,1)]; % Initial condition ?????????? Is is necessary?
        Jceq = [Jceq;...
               zeros(1,N) 1 zeros(1,xDim-N-1);...
               zeros(1,N+1) 1 zeros(1,xDim-N-2)]'; % Matlab uses transpose of Jacobian
    elseif isTargetConstraint == 1
        for k = 0:N-1 % Euler Forward
            ceq(2*k+1,1) = x(N+2*(k+1)+1) - x(N+2*k+1) - delta_t*0.310625*x(N+2*k+2);
            ceq(2*k+2,1) = x(N+2*(k+1)+2) - x(N+2*k+2) - delta_t*1*x(k+1);
            Jceq(2*k+1,:) = [zeros(1,N+2*k) -1 -delta_t*0.310625 1 zeros(1,xDim-N-2*k-3)];
            Jceq(2*k+2,:) = [zeros(1,k) -delta_t zeros(1,N-k-1) zeros(1,2*k+1) -1 0 1 zeros(1,xDim-N-2*k-4)];
        end
        ceq = [ceq;x(N+1,1)+1; x(N+2,1);x(xDim-1,1)-1;x(xDim,1)]; % Initial condition ?????????? Is is necessary?
        Jceq = [Jceq;...
               zeros(1,N) 1 zeros(1,xDim-N-1);...
               zeros(1,N+1) 1 zeros(1,xDim-N-2);...
               zeros(1,xDim-2) 1 0;...
               zeros(1,xDim-1) 0]'; % Matlab uses transpose of Jacobian
    end
           
end