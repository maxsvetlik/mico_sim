function [rwd] = mico_sim(q_t, ts)
deg = pi/180;
home = deg*[274.85, 175.147, 78.915, 243.27, 83.9318, 75.136]*deg; %Kinova home position
mdl_mico;
mico = evalin('base','mico');
p_tar = [.210209, -.2619, .4771]; %c_tar should be zone
q_c = home;
t = [0:1:ts];
[q,qd,qdd] = jtraj(q_c, q_t, ts);
tau = mico.rne(q,qd,qdd);
taug = mico.gravload(q);
tnet = tau - taug;
tnet(ts - 1)
end