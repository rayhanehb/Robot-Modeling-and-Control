function q = inverse(H, myrobot)
    R_d = H(1:3, 1:3);
    o_d = H(1:3, 4);

    o_c = o_d - R_d*[0; 0; myrobot.d(6)];

    x_c = o_c(1);
    y_c = o_c(2);
    z_c = o_c(3);

    theta1 = atan2(y_c, x_c) - atan2(-myrobot.d(2), real(sqrt(x_c^2 + y_c^2 - myrobot.d(2)^2)));

    D = (x_c^2 + y_c^2 - myrobot.d(2)^2 + (z_c - myrobot.d(1))^2 - myrobot.a(2)^2 - myrobot.d(4)^2) / (2*myrobot.a(2)* myrobot.d(4));
    theta3 = atan2(D, sqrt(1 - D^2));

    theta2 = atan2(z_c - myrobot.d(1), real(sqrt(x_c^2 + y_c^2 - myrobot.d(2)^2))) - atan2(-myrobot.d(4)*cos(theta3), myrobot.a(2)+myrobot.d(4)*sin(theta3));

    H_3_0 = eye(4);
    joint = [theta1 theta2 theta3];

    for i = 1:3
        H_i = [cos(joint(i)), -sin(joint(i))*cos(myrobot.alpha(i)), sin(joint(i))*sin(myrobot.alpha(i)), myrobot.a(i)*cos(joint(i));
               sin(joint(i)), cos(joint(i))*cos(myrobot.alpha(i)), -cos(joint(i))*sin(myrobot.alpha(i)), myrobot.a(i)*sin(joint(i));
               0, sin(myrobot.alpha(i)), cos(myrobot.alpha(i)), myrobot.d(i);
               0, 0, 0, 1];

        H_3_0 = H_3_0*H_i;
    end

    R_3_0 = H_3_0(1:3, 1:3);
    R_3_6 = R_3_0'*R_d;

    theta4 = atan2(R_3_6(2,3), R_3_6(1,3));
    theta5 = atan2(real(sqrt(1 - R_3_6(3,3)^2)), R_3_6(3,3));
    theta6 = atan2(R_3_6(3,2), -R_3_6(3,1));

    q = [theta1 theta2 theta3 theta4 theta5 theta6];
end
% %Part 4.4
% function q = inverse(H, myrobot)
%     Rd = H(1:3, 1:3);
%     O_d = H(1:3, 4);
%     O_c = O_d - Rd*[0; 0; myrobot.d(6)];    % origin of the wrist
% 
%     xc = O_c(1);
%     yc = O_c(2);
%     zc = O_c(3);
%     s = zc - myrobot.d(1);
%     d = myrobot.d;
%     a = myrobot.a;
%     alphas = myrobot.alpha;
%     r = sqrt(xc^2 + yc^2 - d(2)^2);                         
% 
%     D = (r^2 + s^2 - a(2)^2 - d(4)^2)/(2 * a(2) * d(4));
%     q(3) = atan2(D, real(sqrt(1 - D^2)));
% 
%     q(1) = atan2(yc, xc) - atan2(d(2),r);
%     q(2) = atan2(s, r) - atan2(d(4)*sin(q(3) - pi/2), a(2) + d(4)*cos(q(3) - pi/2));
% 
%     % Euler: A = R_0_to_3
%     H3 = eye(4);
% 
%     for i = 1:3 % Finding H 0 to 3
%         H3_hat = [cos(q(i)) -sin(q(i))*cos(alphas(i)) sin(q(i))*sin(alphas(i)) a(i)*cos(q(i));
%                 sin(q(i)) cos(q(i))*cos(alphas(i)) -cos(q(i))*sin(alphas(i)) a(i)*sin(q(i));
%                 0 sin(alphas(i)) cos(alphas(i)) d(i);
%                 0 0 0 1];
%         H3 = H3*H3_hat;
%     end
% 
%     R_0_to_3 = H3(1:3,1:3);
%     A = R_0_to_3'*Rd;   %R_3_to_6
%     
% 
%     q(4) = atan2(A(2,3), A(1,3));
%     q(5) = atan2(sqrt(1 - A(3,3)^2), A(3,3));
%     q(6) = atan2(A(3,2), A(3,1));
% end