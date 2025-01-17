% define the PUMA560 robot
DH =    [0 76      0    pi/2;
        0  -23.65   43.23  0;
        0  0       0    pi/2;
        0  43.18   0    -pi/2;
        0  0       0    pi/2;
        0  20      0    0];


myrobot = mypuma560(DH);

Htest = [cos(pi/4) -sin(pi/4) 0 20;
        sin(pi/4) cos(pi/4) 0 23;
        0 0 1 15;
        0 0 0 1];
qtest = inverse(Htest, myrobot);

% test the attractive force code
H1 = eul2tr([0 pi pi/2]); % eul2tr converts ZYZ Euler angles to a hom. tsf. mtx
H1(1:3,4)=100*[-1; 3; 3;]/4; % This assigns the desired displacement to the hom.tsf.mtx.
q1 = inverse(H1,myrobot);
% This is the starting joint variable vector.
H2 = eul2tr([0 pi -pi/2]);
H2(1:3,4)=100*[3; -1; 2;]/4;
q2 = inverse(H2,myrobot);
% This is the final joint variable vector
tau = att(q1,q2,myrobot)


% plot without any obstacles
qref = motionplan(q1,q2,0,10,myrobot,[],0.01);
t=linspace(0,10,300);
q = ppval(qref,t)';
plot(myrobot,q)

% test the repulstive force code
setupobstacle
q3 = 0.9*q1+0.1*q2;
tau = rep(q3,myrobot,obs{1}) % This tests the torque for the cylinder obstacle

% this tests for the spherical obstacle
q = [pi/2 pi 1.2*pi 0 0 0];
tau = rep(q,myrobot,obs{6})

% plot the whole trajectory
setupobstacle
hold on
plot3(-25, 75, 75, '+k', 'MarkerSize',10)
plot3(75, -25, 50, '+k', 'MarkerSize',10)
axis([-100 100 -100 100 0 200])
view(-32,50)
plotobstacle(obs);
qref = motionplan(q1,q2,0,10,myrobot,obs,0.01);
t=linspace(0,10,300);
q=ppval(qref,t)';
plot(myrobot,q);
hold off


