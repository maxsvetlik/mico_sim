%switch sim
function switch_sim
global switch_origin switch_dim switch_angle switch_limits qdd_limits
switch_origin = [.2, .2, 1]; %meters, xyz
switch_dim = [.01, .01, .02]; %meters, length witdth height
switch_angle = 45; %degrees, defines switch angle
switch_limits = [-switch_angle switch_angle];
mdl_mico;
reward = 0;
home = evalin('base','home');
%simulation parameters
episodes = 1;
timesteps = 3;
q_f = home; %define initial q vector
qdd_limits = [2, 2, 2, 2, 2, 2];


for ep = 1:episodes
    for timestep = 1:timesteps
        xd = input('Enter a velocity command: [dx dy dz drx dry drz]: ');
        [q_f, qd_f, qdd_f, x_f, tau] = mico_sim(q_f, xd);
        reward = reward + valuefun(qdd_f, x_f, tau, xd);
        %disp(qdd_f);
    end
end
reward
end

function reward = valuefun(qdd_f, x_f, tau, xd)
global switch_origin switch_dim switch_angle qdd_limits
reward = -500;
death = -1000;
%check if joint accelerations exceed limits
for i = 1:size(qdd_limits)    
    if abs(qdd_f(1,i)) > qdd_limits(1,i)
        qdd_f(i)
        qdd_limits(i)
        reward = death;
    end
end
%check if EF at light
xrange = x_f(4,1) > switch_origin(1)-switch_dim(1) & x_f(4,1) < switch_origin(1) + switch_dim(1);
yrange = x_f(4,2) > switch_origin(2)-switch_dim(2) & x_f(4,2) < switch_origin(2) + switch_dim(2);
zrange = x_f(4,3) > sin(switch_angle)*switch_origin(3)-switch_dim(3) & x_f(4,3) < sin(switch_angle)*switch_origin(3) + switch_dim(3);
if xrange & yrange & zrange
   %update reward with amount of movement caused to switch
   reward = switch_movement(tau, xd)
end
end


%models the movement of the lightswitch as a simple spring
%assumes last element of tau vector is differential force of ef
function x = switch_movement(tau, xd)
global switch_dim switch_angle switch_limits
tau_ef = tau(size(tau));
alpha = arctan(xd(2)/xd(3)); %the resultant angle of striking the switch
k = 6.5 %N/m :- arbitrary spring constant
tau_effective = tau_ef*(switch_angle/alpha);
x = sqrt(tau_effective/k);
max_dist = 2 * switch_dim(z) * 2 * pi * switch_limits(2);
if x > max_dist
   x = -1000;
else if x >= .5*max_dis %assume that at half mast, the switch flips all the way
    x = 0;
    end
end
end