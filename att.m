% This is our original code - we could not figure out why it does not work
% function tau = att(q, q2, myrobot)
%     % q: column vector of actual joint angles
%     % q2: column vector of final joint angles
%     % q = q';
%     % q2 = q2 + [ 0 0 0 2*pi 0 0;
%     % compute forward kinematics for current/final position o_i
%     thetas = q;         % 1x6
%     thetas2 = q2;  
%     alphas = myrobot.alpha; % 
%     as = myrobot.a;
%     ds = myrobot.d;
%     H = eye(4);
%     H_bar = eye(4);
%     for i = 1:6
%         Hi = [cos(thetas(i)) -sin(thetas(i))*cos(alphas(i)) sin(thetas(i))*sin(alphas(i)) as(i)*cos(thetas(i));
%                         sin(thetas(i)) cos(thetas(i))*cos(alphas(i)) -cos(thetas(i))*sin(alphas(i)) as(i)*sin(thetas(i));
%                         0 sin(alphas(i)) cos(alphas(i)) ds(i);
%                         0 0 0 1];
%         H = H*Hi;
%         o(:, i) = H(1:3,4);
%         disp(o);
%         z(:, i) = H(1:3,3);
% 
%         Hi_bar = [cos(thetas2(i)) -sin(thetas2(i))*cos(alphas(i)) sin(thetas2(i))*sin(alphas(i)) as(i)*cos(thetas2(i));
%                         sin(thetas2(i)) cos(thetas2(i))*cos(alphas(i)) -cos(thetas2(i))*sin(alphas(i)) as(i)*sin(thetas2(i));
%                         0 sin(alphas(i)) cos(alphas(i)) ds(i);
%                         0 0 0 1];
%         H_bar = H_bar*Hi_bar;
%         o_bar(:, i) = H_bar(1:3,4)
% 
%     end
% 
%     % compute artificial forces 
%     c = 1;
%     % for i = 1:6
%     %     F(:,i) = -c * (o(i,:) - o_bar(i,:));
%     % end
% 
%     % compute Jacobians (all R joints) & normalize
%     tau = 0;
%     for i = 1:6
%         Fi = -c * (o(:, i) - o_bar(:, i))
%         if i == 1
%             Jv(:, i) = cross([0 0 1]',(o(:, 1)));
%             Joi = [Jv(:, i) zeros(3,6-i)];
%             tau = tau + Joi'*Fi;
%         else
%             z(:, i) = H(1:3,3);
%             for j = 1:i
%               Jv(:, i) = cross(z(:, i-1),(o(:, j) - o(:, i-1)));
%               Joi = [Jv(:, 1:i) zeros(3,6-i)];
%               tau = tau + Joi'*Fi;
%             end
%             % Joi = [Jv(:, 1:i) zeros(3,6-i)];
%             % tau = tau + Joi'*Fi;
%         % end
%     end
% 
%     % tau = 0;
%     % for i = 1:6
%     %     tau = tau + Jv(i,:)'*F(i,:);
%     % end
% 
% end



% This is the code that was given to us after we could not figure out why
% ours was broken
function tau = att(q, q2, myrobot)
    o = zeros(3,7);
    of = zeros(3,7);
    Jv = zeros(3,6,6);
    z = zeros(3,6);
    H = eye(4,4);
    Hf = eye(4,4);
    zeta = 1*ones(6,1);
    tau = zeros(6,1);
    
    % loop through the number of joints in our manipulator
    for i = 1:6
        % let z(:, i) be the z axis for joint i-1
        z(:,i) = H(1:3, 3);
        joint = q;
        % calculate the forward kinematics to get the origin of joint i
        A_i = [cos(joint(i)), -sin(joint(i))*cos(myrobot.alpha(i)), sin(joint(i))*sin(myrobot.alpha(i)), myrobot.a(i)*cos(joint(i));
               sin(joint(i)), cos(joint(i))*cos(myrobot.alpha(i)), -cos(joint(i))*sin(myrobot.alpha(i)), myrobot.a(i)*sin(joint(i));
               0, sin(myrobot.alpha(i)), cos(myrobot.alpha(i)), myrobot.d(i);
               0, 0, 0, 1];
        H = H*A_i;
        % let o(:, i) be the origin for joint i-1
        o(:, i+1) = H(1:3, 4);
        
        % calculate the forward kinematics to get the desired origin of
        % joint i
        joint = q2;
        A_if = [cos(joint(i)), -sin(joint(i))*cos(myrobot.alpha(i)), sin(joint(i))*sin(myrobot.alpha(i)), myrobot.a(i)*cos(joint(i));
               sin(joint(i)), cos(joint(i))*cos(myrobot.alpha(i)), -cos(joint(i))*sin(myrobot.alpha(i)), myrobot.a(i)*sin(joint(i));
               0, sin(myrobot.alpha(i)), cos(myrobot.alpha(i)), myrobot.d(i);
               0, 0, 0, 1];
        Hf = Hf*A_if;
        of(:,i+1) = Hf(1:3, 4);
        
        % calculate the jacobian
        for j=1:6
            if j<=i
                Jv(:,j,i) = cross( z(:,j), ( o(:,i+1) - o(:,j) ) );
            end
        end
        
        % calculate the artificial attractive force on each joint
        F = -zeta(i)*(o(:,i+1) - of(:, i+1));
        
        % calculate the torques on each joint resulting from the attractive
        % force
        tau = tau + Jv(:,:,i)'*F; 
    end
    
    % normalize the torques (we want to take many small steps, so we only
    % care about their direction)
    % later we will scale this by the step size
    tau = tau';
    if norm(tau) ~= 0
        tau = tau/norm(tau);
    end
end