function [utraj,xtraj,prog,r] = runWindDircol


r = CrazyflieWindModel();
r.ellipsoidcenter = [2 0 1];
v = constructVisualizer(r.manip);

disp('using crazyflie model in wind!')

N = 21;
minimum_duration = 3;
maximum_duration = 6;
prog = DircolTrajectoryOptimization(r,N,[minimum_duration maximum_duration]);

x0 = Point(getStateFrame(r));

x0.z = 1.01;
x0.x = -1.0;

%!echo "0" > abort.txt
prog = addPlanVisualizer(r,prog);

v.draw(0,double(x0));

prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1); % DirectTrajectoryOptimization method

[ground,world,Ncell] = createGroundBoundary(r,N);
prog = prog.addStateConstraint(BoundingBoxConstraint(double(ground),double(world)),Ncell);

u0 = [0 0 0 0 0 0 r.nominal_thrust]';
prog = prog.addInputConstraint(ConstantConstraint(u0),1); % DirectTrajectoryOptimization method

xf = x0;                       % final conditions: translated in x
upperxf = x0;
lowerxf = x0;

% % Don't move
upperxf.x = x0.x + 1;                 
upperxf.z = x0.z;            
upperxf.mytime = maximum_duration;

lowerxf.x = x0.x + 1;                 
lowerxf.z = x0.z;           
lowerxf.mytime = minimum_duration;


% % Actually move
% upperxf.x = 3;                 % translate x
% upperxf.z = 1;                 % translate z
% upperxf.y = 0;                 % translate x
% upperxf.mytime = maximum_duration;
% 
% lowerxf.x = 3;                 % translate x
% lowerxf.z = 1;                 % translate z
% lowerxf.y = 0;                 % translate x
% lowerxf.mytime = minimum_duration;



prog = prog.addStateConstraint(BoundingBoxConstraint(double(lowerxf),double(upperxf)),N);
%prog = prog.addInputConstraint(ConstantConstraint(u0),N);
prog = prog.addRunningCost(@cost);
prog = prog.addFinalCost(@finalCost);

tf0 = 2;                                                          % initial guess at duration
traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)])); % traj.init.x is a PPTrajectory < Trajectory < DrakeSystem
traj_init.u = ConstantTrajectory(u0);                             % traj_init.u is a ConstantTrajectory < Trajectory < DrakeSystem

info=0;
while (info~=1)
  tic
  %snseti ('Verify level', 3);
  [xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init); % DirectTrajectoryOptimization method
  toc
end

if (nargout<1)
  xtraj = xtraj(1:12)
  xtraj = xtraj.setOutputFrame(r.manip.getStateFrame);      
  v.playback(xtraj,struct('slider',true));
end


% % CREATE TVLQR CONTROLLER
% 
% tic;
% 
% x0 = xtraj.eval(0);
% tf = utraj.tspan(2);
% % Q = 10*eye(13);
% Q = 10 * [eye(3) zeros(3,9) ; zeros(9,3) zeros(9)];
% R = eye(4);
% xtraj = xtraj(1:12);
% 
% %Qf = 10*eye(13);
% 
% % % The following 12 x 12 S matrix was produced from the QuadPlantPenn via:
% % x0 = Point(getStateFrame(r));
% % x0.z = 1.01; % lift the quad off the ground
% % x0.x = -1.0;
% % upperxf = x0;
% % upperxf.x = 3;                 % translate x
% % upperxf.z = 1;                 % translate z
% % upperxf.y = 0;                 % translate x
% % [one, two] = tilqr(r,upperxf,double(nominalThrust(r)),10*eye(12),eye(4));
% % two.S;
% 
% %    14.5445   -0.0000   -0.0000    0.0000   10.1595   -0.0000    5.5772   -0.0000   -0.0000    0.0000    0.0294   -0.0000
% %    -0.0000   14.5445    0.0000  -10.1595   -0.0000   -0.0000    0.0000    5.5772    0.0000   -0.0294   -0.0000   -0.0000
% %    -0.0000    0.0000   10.7616   -0.0000   -0.0000   -0.0000   -0.0000    0.0000    0.7906   -0.0000   -0.0000   -0.0000
% %     0.0000  -10.1595   -0.0000   55.1653    0.0000    0.0000    0.0000  -14.7471   -0.0000    0.1608    0.0000    0.0000
% %    10.1595   -0.0000   -0.0000    0.0000   55.1653   -0.0000   14.7471   -0.0000    0.0000    0.0000    0.1608   -0.0000
% %    -0.0000   -0.0000   -0.0000    0.0000   -0.0000   10.2549   -0.0000   -0.0000   -0.0000    0.0000   -0.0000    0.2581
% %     5.5772    0.0000   -0.0000    0.0000   14.7471   -0.0000    7.0761   -0.0000    0.0000    0.0000    0.0427   -0.0000
% %    -0.0000    5.5772    0.0000  -14.7471   -0.0000   -0.0000   -0.0000    7.0761   -0.0000   -0.0427   -0.0000   -0.0000
% %    -0.0000    0.0000    0.7906   -0.0000    0.0000   -0.0000    0.0000   -0.0000    0.8508   -0.0000    0.0000    0.0000
% %     0.0000   -0.0294   -0.0000    0.1608    0.0000    0.0000    0.0000   -0.0427   -0.0000    0.0299    0.0000   -0.0000
% %     0.0294   -0.0000   -0.0000    0.0000    0.1608   -0.0000    0.0427   -0.0000    0.0000    0.0000    0.0299   -0.0000
% %    -0.0000   -0.0000   -0.0000    0.0000   -0.0000    0.2581   -0.0000   -0.0000    0.0000   -0.0000   -0.0000    0.2647
% 
% Qf = [   14.5445   -0.0000   -0.0000    0.0000   10.1595   -0.0000    5.5772   -0.0000   -0.0000    0.0000    0.0294   -0.0000;
%    -0.0000   14.5445    0.0000  -10.1595   -0.0000   -0.0000    0.0000    5.5772    0.0000   -0.0294   -0.0000   -0.0000;
%    -0.0000    0.0000   10.7616   -0.0000   -0.0000   -0.0000   -0.0000    0.0000    0.7906   -0.0000   -0.0000   -0.0000;
%     0.0000  -10.1595   -0.0000   55.1653    0.0000    0.0000    0.0000  -14.7471   -0.0000    0.1608    0.0000    0.0000;
%    10.1595   -0.0000   -0.0000    0.0000   55.1653   -0.0000   14.7471   -0.0000    0.0000    0.0000    0.1608   -0.0000;
%    -0.0000   -0.0000   -0.0000    0.0000   -0.0000   10.2549   -0.0000   -0.0000   -0.0000    0.0000   -0.0000    0.2581;
%     5.5772    0.0000   -0.0000    0.0000   14.7471   -0.0000    7.0761   -0.0000    0.0000    0.0000    0.0427   -0.0000;
%    -0.0000    5.5772    0.0000  -14.7471   -0.0000   -0.0000   -0.0000    7.0761   -0.0000   -0.0427   -0.0000   -0.0000;
%    -0.0000    0.0000    0.7906   -0.0000    0.0000   -0.0000    0.0000   -0.0000    0.8508   -0.0000    0.0000    0.0000;
%     0.0000   -0.0294   -0.0000    0.1608    0.0000    0.0000    0.0000   -0.0427   -0.0000    0.0299    0.0000   -0.0000;
%     0.0294   -0.0000   -0.0000    0.0000    0.1608   -0.0000    0.0427   -0.0000    0.0000    0.0000    0.0299   -0.0000;
%    -0.0000   -0.0000   -0.0000    0.0000   -0.0000    0.2581   -0.0000   -0.0000    0.0000   -0.0000   -0.0000    0.2647];
% 
% %Qf = [Qf zeros(12,1) ; zeros(1,13)];
% %Qf(13,13) = 1e-6;
% 
% r = QuadWindPlant_numerical_12();
% r.windfield = 'flyingsphere';
% r.ellipsoidcenter = [2 0 1];
% xtraj
% utraj
% xtraj = xtraj.setOutputFrame(r.getStateFrame);
% utraj = utraj.setOutputFrame(r.getInputFrame);
% 
% disp('Computing stabilizing controller with TVLQR...');
% ltvsys = tvlqr(r,xtraj,utraj,Q,R,Qf);
% 
% 
% % Optional: CREATE SIMULATION PLANT
% 
% %r2 = QuadWindPlant();
% %r2 = r2.setOutputFrame(r.getOutputFrame);
% %r2 = r2.setInputFrame(r.getInputFrame);
% 
% % CREATE FEEDBACK CONTROLLER
% 
% % For no Gaussian noise (simulating on same plant):
% %sys = feedback(r,ltvsys);
% 
% % For Gaussian noise (simulating on different plant):
% sys = feedback(r,ltvsys);
% 
% toc;
% disp('done!');
% 
% % Cascade if desired
% %tic;
% %sys = cascade(utraj, r);
% %toc;
% 
% % Simulate the result
% tic;
% disp('Simulating the system...');
% x0 = Point(getStateFrame(r));  % initial conditions: all-zeros
% x0.z = 1.01; % lift the quad off the ground
% x0.x = -1.0;
% xtraj_sim = simulate(sys,[0 tf],x0);
% toc;
% disp('done!');
% 
% % Draw the TVLQR result
% xtraj_sim = xtraj_sim(1:12);
% xtraj_sim = xtraj_sim.setOutputFrame(r_temp.getStateFrame());
% v.playback(xtraj_sim, struct('slider', true));
% 
% 
% % % Draw the original result
% % if (nargout<1)
% %    xtraj = xtraj(1:12);
% %    xtraj = xtraj.setOutputFrame(r_temp.getStateFrame());
% %    v2.playback(xtraj,struct('slider',true));
% % end
% 
% 
% 
% 
% % PLOT WIND
% %tiWindDraw(r.windfield);


end

function [g,dg] = cost(dt,x,u)
  R = eye(7);
  g = u'*R*u;
  dg = [0,zeros(1,size(x,1)),2*u'*R];
end

function [h,dh] = finalCost(t,x)
  h = t;
  dh = [1,zeros(1,size(x,1))];
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



