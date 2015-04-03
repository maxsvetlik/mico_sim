%switch sim
function switch_sim
global switch_origin switch_dim switch_angle switch_limits qdd_limits push fall
switch_origin = [-.0142, -.4418, .04]; %meters, xyz
switch_dim = [.01, .01, .02]; %meters, length witdth height
switch_angle = 45; %degrees, defines switch angle
switch_limits = [-switch_angle switch_angle];
mdl_mico;
reward = 0;
writerdy = evalin('base','writerdy');
home = evalin('base','home');

%simulation parameters
episodes = 1;
timesteps = 3;
q_f = writerdy; %define initial q vector
qdd_limits = [2 2 2 2 2 2];


for ep = 1:episodes
    for timestep = 1:timesteps
        xd = input('Enter a velocity command: [dx dy dz drx dry drz]: ');
        q_tm1 = q_f;
        [q_f, qd_f, qdd_f, x_f, tau] = mico_sim(q_f, xd);
        reward = reward + valuefun(qdd_f, x_f, tau, xd);
        if fall
            q_f = fallthrough(q_f, x_f);
            fall = 0;
        else if push
            q_f = pushback(q_tm1);
            push = 0;
            end
        end
        
    end
end
reward
end

function reward = valuefun(qdd_f, x_f, tau, xd)
global switch_origin switch_dim switch_angle qdd_limits
reward = -1;
death = -100;
%check if joint accelerations exceed limits
for i = 1:size(qdd_limits)
    qdd_f
    qdd_limits
    if abs(qdd_f(1,i)) > qdd_limits(1,i)
        reward = death;
    end
end
%check if EF at light
xrange = x_f(4,1) > switch_origin(1)-switch_dim(1) & x_f(4,1) < switch_origin(1) + switch_dim(1);
yrange = x_f(4,2) > switch_origin(2)-switch_dim(2) & x_f(4,2) < switch_origin(2) + switch_dim(2);
zrange = x_f(4,3) > sin(switch_angle)*switch_origin(3)-switch_dim(3) & x_f(4,3) < sin(switch_angle)*switch_origin(3) + switch_dim(3);
if xrange & yrange & zrange
   %update reward with amount of movement caused to switch
   reward = 10/switch_movement(tau, xd)
   disp(reward)
end
end


%models the movement of the lightswitch as a simple spring
%assumes last element of tau vector is differential force of ef
function x = switch_movement(tau, xd)
global switch_dim switch_angle switch_limits push fall
tau_ef = sum(tau);%tau(size(tau));
alpha = arctan(xd(2)/xd(3)); %the resultant angle of striking the switch
k = .5 %N/m :- arbitrary spring constant
tau_switch = tau_ef*(switch_angle/alpha);
x = sqrt(tau_switch/k);
disp(x);
disp(tau_switch);
max_dist = 2 * switch_dim(z) * 2 * pi * switch_limits(2);
if x > max_dist
   x = -1000;
   push = 1;
else if x >= .5*max_dis %assume that at half mast, the switch flips all the way
    x = 10;
    fall = 1;
    end
end
end
%function to simulate the force back from the switch if not engaged
function q_t = pushback(q_tm1)
%since mico_sim is discrete, just reset the q_t to that of t-1
    q_t = q_tm1;
end
%function to simulate the displacement seen by engaging the switch
function q_t = fallthrough(q, x)
global switch_dim switch_angle
mico = evalin('base','mico');
arclength = switch_dim(3)*switch_angle*pi/180;
x_final = transl(x(1,4),x(2,4),-arclength+x(3,4));
q_t = mico.ikinem(x_final,q,'qlimits','pweight',100);
end