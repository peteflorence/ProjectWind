function [utraj,xtraj,prog,r] = runPennDircolWind

r_temp = Quadrotor();
%r_temp = addOcean(r_temp, [.8,.45,1.25], [.20;2.5], pi/4);
v = constructVisualizer(r_temp);
r = QuadWindPlant(); % Quadrotor constructor
r.windfield = 'constant';
r.ellipsoidcenter = [3 0 1];



disp('using quad plant in wind based on penn plant!')

N = 21;
minimum_duration = .1;
maximum_duration = 3;
prog = DircolTrajectoryOptimization(r,N,[minimum_duration maximum_duration]);
x0 = Point(getStateFrame(r));  % initial conditions: all-zeros

x0.z = 1; % lift the quad off the ground


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

upperxf.x = 5;                 % translate x
upperxf.z = 1;                 % translate z
upperxf.y = 0;                 % translate x
upperxf.mytime = maximum_duration;

lowerxf.x = 5;                 % translate x
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

% Optional: CREATE SIMULATION PLANT

r2 = QuadWindPlant();
r2 = r2.setOutputFrame(r.getOutputFrame);
r2 = r2.setInputFrame(r.getInputFrame);

% CREATE FEEDBACK CONTROLLER

% For no Gaussian noise (simulating on same plant):
%sys = feedback(r,ltvsys);

% For Gaussian noise (simulating on different plant):
sys = feedback(r2,ltvsys);

toc;
disp('done!');

% Cascade if desired
%tic;
%sys = cascade(utraj, r);
%toc;

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


% % Draw the original result
% if (nargout<1)
%    xtraj = xtraj(1:12);
%    xtraj = xtraj.setOutputFrame(r_temp.getStateFrame());
%    v2.playback(xtraj,struct('slider',true));
% end




% PLOT WIND
tiWindDraw(r.windfield);


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
%dh = [1,zeros(1,size(x,1)-1),1]; % is this right?

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



