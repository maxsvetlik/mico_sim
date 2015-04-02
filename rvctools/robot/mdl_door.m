% mdl_door is a doorhandle modeled as a robot manipulator
% 
% This model is used as an extension of Peter Corke's robot toolbox.
% The RTB is distributed under the GNU Leser General Public License.
% http://www.petercorke.com

function r = mdl_jaco()
    
    deg = pi/180;
    
    % robot length (meters)
    l = .2;

    robot = SerialLink([
        Revolute('alpha', 0,  'a', 0,  'd', 0)
        Revolute('alpha', pi/2,  'a', 0,  'd', l )
        ], ...
        'name', 'DoorHandle', 'manufacturer', 'MaxLabs');

    
    % place the variables into the global workspace
    if nargin == 1
        r = robot;
    elseif nargin == 0
        assignin('base', 'handle', robot);
        assignin('base', 'qz', [0 0 0 0 0 0]); % zero angles
        assignin('base', 'qr', [270 180 180 0 0 0]*deg); % vertical pose as per Fig 2
    end
end
