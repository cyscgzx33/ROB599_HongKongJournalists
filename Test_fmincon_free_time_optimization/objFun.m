function [J,Jacobian_J] = objFun(z,sysParam)
    zDim = sysParam.zDim;
    zDim_per_step = sysParam.zDim_per_step;
    NumStates = sysParam.NumStates;
    isTargetConstraint = sysParam.isTargetConstraint;
    TargetPos = sysParam.TargetPos;
    TargetPosWeight = sysParam.TargetPosWeight;
    
    if isTargetConstraint == 0  % Target as a part of the cost function
        J = 0; 
        Jacobian_J = zeros(zDim,1); % Matlab uses Column vector for Jacobian
        J = z(zDim,1).^2 + TargetPosWeight * norm(z(zDim-zDim_per_step+1+1:zDim-zDim_per_step+1+NumStates,1)-TargetPos); 
        Jacobian_J(zDim,1) = 2*z(zDim,1);
        Jacobian_J(zDim-zDim_per_step+1+1:zDim-zDim_per_step+1+NumStates,1) = 2*TargetPosWeight.*(z(zDim-zDim_per_step+1+1:zDim-zDim_per_step+1+NumStates,1)-TargetPos); 
    elseif isTargetConstraint == 1 % Target as constraint, so it does not appear in cost function J
        J = 0; 
        Jacobian_J = zeros(zDim,1); % Matlab uses Column vector for Jacobian
        J = z(zDim,1).^2;
        Jacobian_J(zDim,1) = 2*z(zDim,1);
    end
    
end