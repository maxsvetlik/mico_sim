%Demonstration using the mico model
%Using robot toolkit
%Note: if having issues, run startup_rvc.m to set path

deg = pi/180;

syms q1 q2 q3 q4 q5 q6;
symbolic = [q1,q2,q3,q4,q5,q6];
home = [274.85, 175.147, 78.915, 243.27, 83.9318, 75.136]*deg; %Kinova home position
home_c = [.210209, -.2619, .4771];
grab = [267.574, 218.327, 105.551, 197.795, 109.295, 79.431]*deg; %Grab pen position from draw demo
grab_c = [.2577, -.401, .514];
ground = [240.055, 249.816, 110.29412, 343.90, 49.091, 340.5]*deg; %EF in front of base, touching ground
ground_c = [.14386,-0.36297,0.00080];

%Load model
mdl_mico;

%Move arm to pos
T = mico.fkine(home);
curTHETA = home;
cur_c = home_c;
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
%for i = 1:length(TF)
%    curTHETA = mico.ikunc(transl(temp(i,13),temp(i,14),temp(i,15)));
%    %mico.plot(curTHETA)
%    q = [q;curTHETA];
%end

%T_cur = mico.fkine(curTHETA)
%mico.plot(q);

%T1 = transl(.210209, -.2619, .4771);
%T2 = transl(.2577, -.401, .514);
%TF = ctraj(T1,T2,50);
%q = mico.ikunc(transl(grab_c), home); %THIS combination has showed the best accuracy/speed ratio
q = mico.ikinem(T2,home,'qlimits','pweight',100,'stiffness',.2);
asdf = mico.fkine(q)
mico.plot(q)
%qi = mico.ikine(T, home, 'ilimit', 5000, 'plot', 'alpha', .5, 'tol', 1e-6)
%qi = mico.ikine3(T)
%T_cur