function [q_f1 qd_f] = mico_sim(q_t)
global tsize deg q_0 kp first_run writerdy
deg = pi/180;
home = [274.85, 175.147, 78.915, 243.27, 83.9318, 75.136]*deg; %Kinova home position
writerdy = [262.113, 248.05, 128.54, 343.2272, 72.75, 149.25]*deg;
tsize = .07;
first_run = 1;
q_0 = writerdy; %q naught defines theta at time 0
kp = .5; %proportionality constant
mdl_mico;
mico = evalin('base','mico');

%%%%%%%%%%%%%%%%%
%q_c = home;
%t = [0:1:ts];
%[q,qd,qdd] = jtraj(q_c, q_t, ts);
%fileID = fopen('~/max_arm_exp/simulation/a6.txt','w');
%for i = 1:length(q)
%    fprintf(fileID,'%5f\n',q(i,6)*inv(deg));
%end
%pos_f
%ts([0 0 0 0 0 1]');
q_t1 = writerdy;
mico.fkine(q_t1)
%for i = 1:10
while true
asdf = [0 0 -.2 0 0 0]';
[q_t1, xd_t1] = ts(q_t1, asdf);
end
mico.fkine(q_t1)
end
function [q_t1 xd_t1] = ts(q_t, xd_d)
    global mico home first_run q_0 tsize kp
    mico = evalin('base','mico');
%    q_t = home;
%     if first_run == 1
%         q_t = q_0;
%         first_run = 0;
%     end
    T6 = mico.fkine(q_t);
    %d6 = tr2jac(T6) * xd_d;
    qd_t = inv(mico.jacob0(q_t)) * xd_d; %required joint velocities
    qstar = q_t + (tsize * qd_t');
    qd_t = qd_t';
    %e = qd_t - qd_d; %P controller
    %[tau,q] = mico.fdyn(1, @controller, q_t, qd_t, qstar, kp, tsize);
    tau = mico.rne(q_t, qd_t, kp*(qd_t-q_t)) %singe integration,t = 1
    tau2 = mico.gravload(q_t);
    q_t1 = qstar;
    q_t = q_t1;
    xd_t1 = xd_d;
    mico.plot(q_t);
    end
    function tau = controller(~, t, q_t, qd_t, qstar, P, tsize)
          qstar = q_t + (tsize * qd_t);
          tau = P*(qstar-q_t);
    end
%   0.187508 -0.782107 +0.285158 -0.0863431 -0.0341825-0.0777281 %real
%   -1.0266  -11.1628    3.1100   -1.0630   -0.0930    0.0000 %tau
%   -0.0000  -10.0123    2.8712   -0.9235   -0.0408   -0.0000 %taug

%   [T,Q,QD] = R.fdyn(T, TORQFUN) integrates the dynamics of the robot over 
%   the time  interval 0 to T and returns vectors of time T, joint position Q
%   and joint velocity QD.  The initial joint position and velocity are zero.
%   The torque applied to the joints is computed by the user-supplied control
%   function TORQFUN:
%  
%          TAU = TORQFUN(T, Q, QD)
%  
%   where Q and QD are the manipulator joint coordinate and velocity state 
%   respectively, and T is the current time. 
%  
%   [TI,Q,QD] = R.fdyn(T, TORQFUN, Q0, QD0) as above but allows the initial
%   joint position and velocity to be specified.
%  
%   [T,Q,QD] = R.fdyn(T1, TORQFUN, Q0, QD0, ARG1, ARG2, ...) allows optional 
%   arguments to be passed through to the user-supplied control function:
%  
%          TAU = TORQFUN(T, Q, QD, ARG1, ARG2, ...)
%  
%   For example, if the robot was controlled by a PD controller we can define
%   a function to compute the control
%  
%           function tau = mytorqfun(t, q, qd, qstar, P, D)
%             tau = P*(qstar-q) + D*qd;
%  
%   and then integrate the robot dynamics with the control
%  
%          [t,q] = robot.fdyn(10, @mytorqfun, qstar, P, D);