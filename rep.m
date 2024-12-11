%  This is our original code - we could not figure out why it does not work

% function tau = rep(q, myrobot, obs)
%     %% return tau = sum( J_oi(q)'*F_{i, rep}(O_{i, 0}(q))
% 
% %     q: column vector of actual joint angles
% %     q2: column vector of final joint angles
% %     q = q';
% %     q2 = q2 + [ 0 0 0 2*pi 0 0;
% %     compute forward kinematics for current/final position o_i
%     thetas = q;         % 1x6  
%     alphas = myrobot.alpha; % 
%     as = myrobot.a;
%     ds = myrobot.d;
%     H = eye(4);
%     eta = 0.1;
%     tau = zeros(6, 1);
%     z = zeros(3, 6);
%     for i = 1:6
% 
%         Hi = [cos(thetas(i)) -sin(thetas(i))*cos(alphas(i)) sin(thetas(i))*sin(alphas(i)) as(i)*cos(thetas(i));
%                         sin(thetas(i)) cos(thetas(i))*cos(alphas(i)) -cos(thetas(i))*sin(alphas(i)) as(i)*sin(thetas(i));
%                         0 sin(alphas(i)) cos(alphas(i)) ds(i);
%                         0 0 0 1];
%         H = H*Hi;
%         o(:, i) = H(1:3,4);
%         z(:, i) = H(1:3,3)
%         disp(o);
% 
% 
%         % Calculate the forces
%         if strcmp(obs.type, 'cyl')
%             disp('force')
%             %% calculate cylinder repulsive force
%             obs.c
%             obs.R
%             oi_c  = o(1:2, i) - obs.c;
%             dist  = norm(oi_c) - obs.R
% %             grad_rho = (1 - obs.R/(norm(oi_c)))*oi_c/(dist);
%             if dist > obs.rho0
%                 F_repi = zeros(3, 1);
%             else
%                 grad_rho = oi_c/norm(oi_c)
%                 F_repi = eta * ((1 / dist) - (1/obs.rho0)) * (1/dist^2) * grad_rho;
%                 F_repi(3) = 0;
%                 disp(F_repi)
%             end
%         else
%             disp('force1')
%             %% calculate sphere repulsive force
% %             obs.c
% %             obs.R
%             oi_c  = o(1:3, i) - obs.c ;
%             dist  = norm(oi_c) - obs.R % ||oi(q) - b||
%             if dist > obs.rho0
%                 F_repi = zeros(3, 1);
%             else
%                 dist_to_edge = (1 - obs.R/(norm(oi_c)))*oi_c; % oi(q) - b
%     %             grad_rho = dist_to_edge/(dist);
%                 grad_rho = oi_c/norm(oi_c)
%                 F_repi = eta * ((1 / dist) - (1/obs.rho0)) * (1/dist^2) * grad_rho;
%                 disp(F_repi)
%             end
%         end
% 
%         % calculate the jacobians
%         disp('jacobian')
%         if i == 1
%             Jv(:, i) = cross([0 0 1]',(o(:, 1)));
%             Joi = [Jv(:, i) zeros(3,6-i)];
%             tau = tau + Joi'*F_repi;
%         else
% %             z(:, i) = H(1:3,3);
%             for j = 1:i
%               Jv(:, i) = cross(z(:, i-1),(o(:, j) - o(:, i-1)));
%               Joi = [Jv(:, 1:i) zeros(3,6-i)];
%               tau = tau + Joi'*F_repi;
%             end
%         end
%     end
%     tau = tau';
%     if norm(tau) ~= 0
%         tau = tau / norm(tau);
%     end
% end


% This is the code that was given to us after we could not figure out why
% ours was broken
function tau = rep(q,myrobot,obs)

    d = myrobot.d;
    alpha = myrobot.alpha;
    a = myrobot.a;
    
    R=obs.R;
    c=obs.c;
    rho0=obs.rho0;

    % partial forward kinematics for jacobian and force calculation
    H = eye(4,4);
    o = zeros(3,7);
    z = zeros(3,6);
    for i=1:6
        % forward kinematics to calculate actual location
        z(:,i) = H(1:3,3);
        A_i = [cos(q(i))    -sin(q(i))*cos(alpha(i))    sin(q(i))*sin(alpha(i))     a(i)*cos(q(i));
              sin(q(i))     cos(q(i))*cos(alpha(i))     -cos(q(i))*sin(alpha(i))    a(i)*sin(q(i));
              0             sin(alpha(i))               cos(alpha(i))               d(i);
              0             0                           0                           1];
        H = H*A_i;
        o(:,i+1) = H(1:3,4);
    end
        
    % Jacobians
    Jv = zeros(3,6,6);
    for i=1:6
        for j=1:6
            if j<=i
                Jv(:,j,i) = cross(z(:,j),o(:,i+1)-o(:,j));
            end
        end
    end
    
    % calcualting artificial torques
    tau = zeros(6,1);
    for i=1:6
        
        % shortest distance to obstacle, 
        switch obs.type
            case 'cyl'
                % ignore z since vector to closest point lies on a
                % transpose of the x-y plane
                rho =  norm(o(1:2,i+1) - c) - R;
            case 'sph'
                rho =  norm(o(1:3,i+1) - c) - R;
        end
        
        %artificial repulsive force (5.5-5.6)
        if rho>rho0
            F = zeros(3,1);
        else
            switch obs.type
                case 'cyl'
                    gradrho = [o(1:2,i+1)-c;0;]/norm([o(1:2,i+1)-c;0;]);
                case 'sph'
                    gradrho = (o(1:3,i+1)-c)/norm(o(1:3,i+1)-c);
            end
            nu = 0.1;
            F = nu*(1/rho-1/rho0)*gradrho/rho^2;
        end
        
        %add artificial joint torques
        tau = tau + Jv(:,:,i)'*F;
        
    end
    
    % normalize the torques (we want to take many small steps, so we only
    % care about their direction)
    % later we will scale this by the step size
    tau = tau';
    if norm(tau)~=0
        tau = tau / norm(tau);
    end
        
end
