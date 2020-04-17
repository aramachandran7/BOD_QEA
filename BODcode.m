% BOD code : )

function BODcode()
% Insert any setup code you want to run here

% define u explicitly to avoid error when using sub functions
% see: https://www.mathworks.com/matlabcentral/answers/268580-error-attempt-to-add-variable-to-a-static-workspace-when-it-is-not-in-workspace

% boilerplate code for starting Neato Position
u = [];
% u will be our parameter
syms u;

% this is the equation of the bridge
R = 4*[0.396*cos(2.65*(u+1.4));-0.99*sin(u+1.4);0];
T = diff(R);
That = T/norm(T);


% Mathematical code for controlling robot and creating wheel velocity
% functions
t = [];
syms t
% syms r
B = .3;
d= .235;
% this is the equation of the bridge
ri=4*.3960*cos(2.65*(B*t+1.4));
rj=4*(-.99)*sin(B*t+1.4);
rk=0*B*t;
r=[ri,rj,rk];
dr=diff(r,t);



T_hat_ugly=dr./norm(dr);
T_hat=simplify(T_hat_ugly);
dT_hat=diff(T_hat,t);
% normalized tangent vector
omega = simplify(cross(T_hat, dT_hat));
speed = simplify(norm(dr));

% setting functions for left and right wheel velocities
vL = speed - (d*(omega(3)))/2;
vR = speed + (d*(omega(3)))/2;

% setting up publishing for ROS to publish to wheel velocities
pub = rospublisher('raw_vel');

% stop the robot if it's going right now
stopMsg = rosmessage(pub);
stopMsg.Data = [0 0];
send(pub, stopMsg);

% starting position for Neato
bridgeStart = double(subs(R,u,0));
startingThat = double(subs(That,u,0));
placeNeato(bridgeStart(1),  bridgeStart(2), startingThat(1), startingThat(2));

% wait a bit for robot to fall onto the bridge
pause(2);

% time to drive!! Calling the runNeato() function defined below
runNeato();


% Run Neato function that walks through a while loop 
function runNeato()
%     grabbing start time to reference during each run of while loop
    start = rostime('now');
    while 1
%         puasing to allow data / encoder collection script to run
        pause(.1);
%         getting time elapsed since start time
       current = rostime('now');
       timeNow = current-start;

%        if mod(seconds(timeNow),3) == 0
%            rostime('now')
%        end 
%        calculating wheel speeds at each point in time
       vLtemp = double(subs(vL,t,seconds(timeNow)));
       vRtemp = double(subs(vR,t,seconds(timeNow)));
%        sending wheel speeds via ROS publishers
       stopMsg.Data = [vLtemp,vRtemp]; 
       send(pub, stopMsg);
%        disp(vLtemp + " " + vRtemp);
       
    end
    
end 
% For simulated Neatos only:
% Place the Neato in the specified x, y position and specified heading vector.
function placeNeato(posX, posY, headingX, headingY)
    svc = rossvcclient('gazebo/set_model_state');
    msg = rosmessage(svc);

    msg.ModelState.ModelName = 'neato_standalone';
    startYaw = atan2(headingY, headingX);
    quat = eul2quat([startYaw 0 0]);

    msg.ModelState.Pose.Position.X = posX;
    msg.ModelState.Pose.Position.Y = posY;
    msg.ModelState.Pose.Position.Z = 1.0;
    msg.ModelState.Pose.Orientation.W = quat(1);
    msg.ModelState.Pose.Orientation.X = quat(2);
    msg.ModelState.Pose.Orientation.Y = quat(3);
    msg.ModelState.Pose.Orientation.Z = quat(4);

    % put the robot in the appropriate place
    ret = call(svc, msg);
end
end