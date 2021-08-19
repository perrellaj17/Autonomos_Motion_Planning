function traj = QuadSim(R,accel,gamma,delta)
%R is the system state vector, where we begin our trajectory analysis from

x = R(1); %X location of the center of robot,
y = R(2); %Y location of the center of robot
t = R(3); %time at this point
theta = R(4); %Orientation of robot, currently radians
v = R(5); %Linear velocity of robot, magnitude 
w = R(6); %Angular velocity of robot, magnitude

%Compute the new location and rotation of robot center
traj(1,1) = x + delta*v*cos(theta);
traj(1,2) = y + delta*v*sin(theta);
%Compute timestep at new node
traj(1,3) = t + delta;

%theta is wrapped between -2Pi and 2Pi
if (theta+ delta*w) >= 0
    traj(1,4) = wrapTo2Pi(theta+ delta*w);
else
    traj(1,4) = -1*wrapTo2Pi(abs(theta+ delta*w));
end

%compute new linear and angular velocity along trajectories
traj(1,5) = v+delta*accel;
traj(1,6) = w+delta*gamma;

%limit the linear and angular velocity of the robot
if traj(1,5) < -5
    traj(1,5) = -5;
elseif traj(1,5) > 5
    traj(1,5) = 5;
end

if traj(1,6) < -pi/2
    traj(1,6) = -pi/2;
elseif traj(1,6) > pi/2
    traj(1,6) = pi/2;
end
