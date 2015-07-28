function [utraj,xtraj,prog,r] = runPennDircolWind_numerical

%r_temp = Quadrotor();
%v = constructVisualizer(r_temp);

r_temp = Quadrotor();
lcmgl = LCMGLClient('Windy');
lcmgl.glColor3f(0,1,0);
v = WindVisualizer(r_temp, lcmgl);
r = QuadWindPlant_numericalCpp(); % Quadrotor constructor
r.windfield = 'flyingsphere';
r.ellipsoidcenter = [2 0 1];



disp('using quad plant in wind based on penn plant!')

N = 21;
minimum_duration = 3;
maximum_duration = 6;
prog = DircolTrajectoryOptimization(r,N,[minimum_duration maximum_duration]);
x0 = Point(getStateFrame(r));  % initial conditions: all-zeros

x0.z = 1.01; % lift the quad off the ground
x0.x = -1.0;
%!echo "0" > abort.txt
prog = addPlanVisualizer(r,prog);

v.draw(0,double(x0));
prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1); % DirectTrajectoryOptimization method

[ground,world,Ncell] = createGroundBoundary(r,N);
prog = prog.addStateConstraint(BoundingBoxConstraint(double(ground),double(world)),Ncell);

u0 = double(nominalThrust(r));
prog = prog.addInputConstraint(ConstantConstraint(u0),1); % DirectTrajectoryOptimization method

xf = x0;                       % final conditions: translated in x
upperxf = x0;
lowerxf = x0;

% % % Don't move
% upperxf.x = x0.x;                 % translate x
% upperxf.z = x0.z;                 % translate z
% upperxf.y = 0;                 % translate x
% upperxf.mytime = maximum_duration;
% 
% lowerxf.x = x0.x;                 % translate x
% lowerxf.z = x0.z;                 % translate z
% lowerxf.y = 0;                 % translate x
% lowerxf.mytime = minimum_duration;


% Actually move
upperxf.x = 3;                 % translate x
upperxf.z = 1;                 % translate z
upperxf.y = 0;                 % translate x
upperxf.mytime = maximum_duration;

lowerxf.x = 3;                 % translate x
lowerxf.z = 1;                 % translate z
lowerxf.y = 0;                 % translate x
lowerxf.mytime = minimum_duration;



prog = prog.addStateConstraint(BoundingBoxConstraint(double(lowerxf),double(upperxf)),N);
prog = prog.addInputConstraint(ConstantConstraint(u0),N);
prog = prog.addRunningCost(@cost);
prog = prog.addFinalCost(@finalCost);

tf0 = 2;                                                          % initial guess at duration
traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)])); % traj.init.x is a PPTrajectory < Trajectory < DrakeSystem
traj_init.u = ConstantTrajectory(u0);                             % traj_init.u is a ConstantTrajectory < Trajectory < DrakeSystem

info=0;
while (info~=1)
  tic
  snseti ('Verify level', 3);
  [xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init); % DirectTrajectoryOptimization method
  toc
end


% CREATE TVLQR CONTROLLER

tic;
x0 = xtraj.eval(0);
tf = utraj.tspan(2);
% Q = 10*eye(13);
Q = 10 * [eye(3) zeros(3,10) ; zeros(10,3) zeros(10)];
R = eye(4);
Qf = 10*eye(13);
disp('Computing stabilizing controller with TVLQR...');
ltvsys = tvlqr(r,xtraj,utraj,Q,R,Qf);
sys = feedback(r,ltvsys);

toc;
disp('done!');

% Simulate the result
tic;
disp('Simulating the system...');
xtraj_sim = simulate(sys,[0 tf],x0);
toc;
disp('done!');

% Draw the TVLQR result
xtraj_sim = xtraj_sim(1:12);
xtraj_sim = xtraj_sim.setOutputFrame(r_temp.getStateFrame());
v.playback(xtraj_sim, struct('slider', true));


end

function [g,dg] = cost(dt,x,u)

R = eye(4);
g = u'*R*u;
%g = sum((R*u).*u,1);
dg = [zeros(1,1+size(x,1)),2*u'*R];
%dg = zeros(1, 1 + size(x,1) + size(u,1));

end

function [h,dh] = finalCost(t,x)

h = t;
dh = [1,zeros(1,size(x,1))]; % original

end

function [ground,world,Ncell] = createGroundBoundary(r,N)

bd = inf;
ground = Point(getStateFrame(r));
ground.x = -bd;
ground.y = -bd;
ground.z = 0.2;
ground.roll = -bd;
ground.pitch = -bd;
ground.yaw = -bd;
ground.xdot = -bd;
ground.ydot = -bd;
ground.zdot = -bd;
ground.rolldot = -bd;
ground.pitchdot = -bd;
ground.yawdot = -bd;
ground.mytime = -bd;
world = Point(getStateFrame(r));
world.x = bd;
world.y = bd;
world.z = bd;
world.roll = bd;
world.pitch = bd;
world.yaw = bd;
world.xdot = bd;
world.ydot = bd;
world.zdot = bd;
world.rolldot = bd;
world.pitchdot = bd;
world.yawdot = bd;
world.mytime = bd;

Ncell = {};
for i = 1:N
  Ncell = [Ncell i];
end

end



