function qref = motionplan(q0,q2,t1,t2,myrobot,obs,tol)

    alpha = 0.025;
    q = q0;
    max_iter = 1;
    
    % keep going until we are close enough to the final point
    while norm(q(end,1:5)-q2(1:5)) >= tol
        % calculate the artificial repulsive and attaractive torques
        tau_rep = zeros(1, 6);
        tau_att = att(q(end, :), q2, myrobot);
        % calculate the repulsive force due to each obstacle
        for i = 1:length(obs)
            tau_rep = tau_rep + rep(q(end, :), myrobot, obs{i});
        end
        % take a step in the direction of the sum of all the repulsive and
        % attractive torques
        q(end+1, :) = q(end, :) + alpha*(tau_att + tau_rep);
        max_iter = max_iter + 1;
        if max_iter > 1000
            break
        end
    end
    % joint 6 can just be linearly interpolated from init to end point
    q(:, 6) = linspace(q0(6), q2(6), size(q, 1));
    % create cubic spline
    t = linspace(t1,t2,size(q,1));
    qref = spline(t,q'); % defines a spline object with interpolation
                         % times in t and interpolation values the columns of q
end