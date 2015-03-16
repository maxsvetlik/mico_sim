%Demonstration using the mico model
%Using robot toolkit
%Note: if having issues, run startup_rvc.m to set path

deg = pi/180;

syms q1 q2 q3 q4 q5 q6;
symbolic = [q1,q2,q3,q4,q5,q6];
home = [274.85, 175.147, 78.915, 243.27, 83.9318, 75.136]*deg; %Kinova home position
%expected: x: 0.210209 y: -0.2619 z: 0.47710
home_c = [.210209, -.2619, .4771];
grab = [267.574, 218.327, 105.551, 197.795, 109.295, 79.431]*deg; %Grab pen position from draw demo
%expected: x: 0.2577 y: -0.401 z: 0.514
grab_c = [.2577, -.401, .514];

%Load model
mdl_mico;

%Move arm to pos
T = mico.fkine(home);
curTHETA = home;
cur_c = home_c;
%setup transform
t = (0 :.2 : 5);
hcn = norm(home_c);
gcn = norm(grab_c);
dx = sqrt(hcn^2 + gcn^2);
T1 = transl(cur_c);
T2 = transl(grab_c);
TF = ctraj(T1,T2,200);


%Pseudo iterative 
%note: ivkin does better with no suggested curTHETA here
%and pseudo iterative does better than large pt. to pt. 
%for i = 1:length(TF)
%    temp = TF(:,:,i);
%    curTHETA = mico.ikunc(temp);
%    mico.plot(curTHETA)
%    T_cur = mico.fkine(curTHETA);

%end
T1 = transl(.210209, -.2619, .4771);
T2 = transl(.2577, -.401, .514);
TF = ctraj(T1,T2,50);
q = mico.ikunc(transl(grab_c), home); %THIS combination has showed the best accuracy/speed ratio
%q = mico.ikine(TF,[1,1,1,1,1,0]);
asdf = mico.fkine(q)
mico.plot(q)
%qi = mico.ikine(T, home, 'ilimit', 5000, 'plot', 'alpha', .5, 'tol', 1e-6)
%qi = mico.ikine3(T)
%T_cur