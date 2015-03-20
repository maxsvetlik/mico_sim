%Demonstration using the mico model
%Using robot toolkit
%Note: if having issues, run startup_rvc.m to set path

function mico_demo
deg = pi/180;

global home home_c grab grab_c ground ground_c ground_t
home = [274.85, 175.147, 78.915, 243.27, 83.9318, 75.136]*deg; %Kinova home position
home_c = [.210209, -.2619, .4771];
grab = [267.574, 218.327, 105.551, 197.795, 109.295, 79.431]*deg; %Grab pen position from draw demo
grab_c = [.2577, -.401, .514];
ground = [240.055, 249.816, 110.29412, 343.90, 49.091, 340.5]*deg; %EF in front of base, touching ground
ground_c = [.14386,-0.36297,0.00080];
ground_t = [.14386,-0.36297,0.00080,.98316,.181175,.01393];
%Load model
mdl_mico;

%call the demo functions
ikine_best;
end
function ex_forward
%Move arm to pos
global home home_c
mico = evalin('base','mico')
T = mico.fkine(home)
J = mico.jacob0(home)
curTHETA = home;
cur_c = home_c;
end
function ivkine_pi
global home ground_c
mico = evalin('base','mico')
%setup transform
t = (0 :.2 : 5);
T1 = transl(cur_c);
T2 = transl(ground_c);
TF = jtraj(T1,T2,50);


%Pseudo iterative 
%note: ivkin does better with no suggested curTHETA here
%and pseudo iterative does better than large pt. to pt.
q = home;
temp = TF(:,:,1);
    for i = 1:length(TF)
        curTHETA = mico.ikunc(transl(temp(i,13),temp(i,14),temp(i,15)));
        %mico.plot(curTHETA)
        q = [q;curTHETA];
    end
end
function ikine_best
global home
mico = evalin('base','mico');
T1 = transl(.210209, -.2619, .4771);
T2 = transl(.2577, -.401, .514);
TF = ctraj(T1,T2,50);
%q = mico.ikunc(transl(grab_c), home); %THIS combination has showed the 2nd best accuracy/speed ratio
q = mico.ikinem(T2,home,'qlimits','pweight',100,'stiffness',.2);
fk = mico.fkine(q)
mico.teach()
end
function vel_cart
%Demonstrating desired cartesianal velocity -> required joint velocity
J2 = mico.jacob0(home)
vel_desired = [.2 .1 .2 .3 0 .004]
inv(J2) * (vel_desired')

%Generate code
%cGen = CodeGenerator(mico)
end