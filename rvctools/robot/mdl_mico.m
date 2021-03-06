%MDL_MICO Create model of Kinova Mico manipulator
%
% MDL_MICO is a script that creates the workspace variable mico which
% describes the kinematic characteristics of a Kinova Mico manipulator
% using standard DH conventions.
%
% Also define the workspace vectors:
%   qz         zero joint angle configuration
%   qr         vertical 'READY' configuration
%
% Reference::
% - "DH Parameters of Mico" Version 1.0.1, August 05, 2013.
%   Kinova
%
% Notes::
% - SI units of metres are used.
% - Unlike most other mdl_xxx scripts this one is actually a function that
%   behaves like a script and writes to the global workspace.
%
% See also SerialLink, Revolute, mdl_jaco, mdl_puma560, mdl_twolink.

% MODEL: Kinova, Mico, 6DOF, standard_DH


% Copyright (C) 1993-2015, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com



% Here is a key of dynamic variables that can be added if they are ever
% discovered:!
%
% Methods::
%  A             link transform matrix
%  RP            joint type: 'R' or 'P'
%  friction      friction force
%  nofriction    Link object with friction parameters set to zero
%  dyn           display link dynamic parameters
%  islimit       test if joint exceeds soft limit
%  isrevolute    test if joint is revolute
%  isprismatic   test if joint is prismatic
%  display       print the link parameters in human readable form
%  char          convert to string
%
% Properties (read/write)::
%
%  theta    kinematic: joint angle
%  d        kinematic: link offset
%  a        kinematic: link length
%  alpha    kinematic: link twist
%  sigma    kinematic: 0 if revolute, 1 if prismatic
%  mdh      kinematic: 0 if standard D&H, else 1
%  offset   kinematic: joint variable offset
%  qlim     kinematic: joint variable limits [min max]
%-
%  m        dynamic: link mass
%  r        dynamic: link COG wrt link coordinate frame 3x1
%  I        dynamic: link inertia matrix, symmetric 3x3, about link COG.
%  B        dynamic: link viscous friction (motor referred)
%  Tc       dynamic: link Coulomb friction
%-
%  G        actuator: gear ratio
%  Jm       actuator: motor inertia (motor referred)

% A note about the masses: These have been adjusted to match the efforts
% reported by an actual Mico manipulator. The officially reported masses
% are 1.4 .78 .78 .78 .4 .4 .4 .4, for the base, joints 1-6, and the hand,
% respectively. Note that these masses do NOT add up to the reported weight
% of the arm by kinova, so...

function r = mdl_mico()
    
    deg = pi/180;
    
    % robot length values (metres)  page 4
    D1 = 0.2755;
    D2 = 0.2900;
    D3 = 0.1233;
    D4 = 0.0741;
    D5 = 0.0741;
    D6 = 0.1600;
    e2 = 0.0070;
    
    % alternate parameters
    aa = 30*deg;
    ca = cos(aa);
    sa = sin(aa);
    c2a = cos(2*aa);
    s2a = sin(2*aa);
    d4b = D3 + sa/s2a*D4;
    d5b = sa/s2a*D4 + sa/s2a*D5;
    d6b = sa/s2a*D5 + D6;
    
    
    % and build a serial link manipulator
    
    % offsets from the table on page 4, "Mico" angles are the passed joint
    % angles.  "DH Algo" are the result after adding the joint angle offset.

    robot = SerialLink([
        Revolute('alpha', pi/2,  'a', 0,  'd', D1,   'flip', ...
        'qlim', [-10000 10000]*deg, ...
	'm', 1.4, ...
    'Jm', .0016 )
        Revolute('alpha', pi,    'a', D2, 'd', 0,    'offset', -pi/2, ...
        'qlim', [50 310]*deg, ...
	'm', .45, ...
    'Jm', .0016  )
        Revolute('alpha', pi/2,  'a', 0,  'd', -e2,  'offset', pi/2, ... 
        'qlim', [35 325]*deg, ...
	'm', .55, ...
    'Jm', .0016  )
        Revolute('alpha', 2*aa,  'a', 0,  'd', -d4b, ...
        'qlim', [-10000 10000]*deg, ... 
	'm', .935, ...
    'Jm', .0016  )
        Revolute('alpha', 2*aa,  'a', 0,  'd', -d5b, 'offset', -pi, ...
        'qlim', [-10000 10000]*deg, ...
	'm', .25, ...
    'Jm', .0016  )
        Revolute('alpha', pi,    'a', 0,  'd', -d6b, 'offset', pi/2, ...
        'qlim', [-10000 10000]*deg, ...
	'm', .71, ...
    'Jm', .0016  )
        ], ...
        'name', 'Mico', 'manufacturer', 'Kinova');
    
    %{
        % MDH version, no test  yet
    robot = SerialLink([
        Revolute('alpha', 0,     'a', 0,  'd', D1,   'modified', 'flip')
        Revolute('alpha', -pi/2, 'a', 0,  'd', 0,    'modified', 'offset', -pi/2)
        Revolute('alpha', 0,     'a', D2, 'd', e2,  'modified',  'offset', pi/2)
        Revolute('alpha', -pi/2, 'a', 0,  'd', d4b, 'modified')
        Revolute('alpha', 2*aa,  'a', 0,  'd', d5b, 'modified',  'offset', -pi)
        Revolute('alpha', 2*aa,  'a', 0,  'd', d6b, 'modified',  'offset', pi/2)
        ], ...
        'name', 'Mico', 'manufacturer', 'Kinova');
    %}
    
    % place the variables into the global workspace
    if nargin == 1
        r = robot;
    elseif nargin == 0
        assignin('base', 'mico', robot);
        assignin('base', 'qz', [0 0 0 0 0 0]); % zero angles, arm up
        assignin('base', 'qr', [270 180 180 0 0 180]*deg); % vertical pose as per Fig 2
        assignin('base', 'test',[260,254.779,102.3429,351.2045,34.9773,246.6818]*deg);
        assignin('base', 'test2',[240.055, 249.816, 110.29412, 343.90, 49.091, 340.5]*deg);
        assignin('base', 'home', [274.85, 175.147, 78.915, 243.27, 83.9318, 75.136]*deg); %Kinova home position
        assignin('base', 'writerdy', [262.113, 248.05, 128.54, 343.2272, 72.75, 149.25]*deg); %Write ready position from draw-demo
        assignin('base', 'zerotau', [0, 180, 180, 0, 0, 180]*deg); %Balanced position, should equate to zero gravitational torques
    end
end

