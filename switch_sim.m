%switch sim
function switch_sim
switch_location = [.2, .2, 1]; %meters, xyz
switch_dim = [.01, .01, .02]; %meters, length witdth height


episodes = 1;
timesteps = 500;
%define initial q vector
qdd_limts = [2 2 2 2 2 2];

global qdd_limits

for ep = 1:episodes
    for timestep = 1: timesteps
        [q_f qd_f qdd_f tau, x_f] = mico_sim(xd);
        reward = valuefun(qdd_f, x_f);
    end
end
end
function r_t = valuefun(qd_f, x_f)
global qdd_limits
reward = -1;
death = -1000;
%check if joint accelerations exceed limits
for i = 1:size(qdd_limits)
    if qdd_f(i) > qdd_limits(i)
        reward = death;
    end
end
%check if EF at light
xrange = x_f(4,1) > switch_location(1)-switch_dim(1) & x_f(4,1) < switch_location(1) + switch_dim(1);
yrange = x_f(4,2) > switch_location(2)-switch_dim(2) & x_f(4,2) < switch_location(2) + switch_dim(2);
zrange = x_f(4,3) > switch_location(3)-switch_dim(3) & x_f(4,3) < switch_location(3) + switch_dim(3);
if xrange & yrange & zrange
   %update reward with amount of movement caused to switch
   reward = switch_movement(tau);
end
end
%models the movement of the lightswitch as a simple spring
%assumes last element of tau vector is differential force of ef
function x = switch_movement(tau)
x = tau(size(tau))
end
