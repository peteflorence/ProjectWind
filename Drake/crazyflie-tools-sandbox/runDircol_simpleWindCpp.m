function [xtraj,utraj,prog] = runDircol_simpleWindCpp()

cfW = CrazyflieWindModelCpp();

lcmgl = LCMGLClient('Windy');
lcmgl.glColor3f(0,1,0);
v = Cf_WindVisualizer(cfW.manip, lcmgl);

N = 11;
minimum_duration = .1;
maximum_duration = 4;
prog = DircolTrajectoryOptimization(cfW,N,[minimum_duration maximum_duration]);  

x0 = Point(getStateFrame(cfW));
x0.z = 1.01; % lift the quad off the ground
x0.x = -1.0;
u0 = [0 0 0 0 0 0 cfW.nominal_thrust]';

prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1);
prog = prog.addInputConstraint(ConstantConstraint(u0),1);

prog = addPlanVisualizer(cfW,prog);
v.draw(0,double(x0));

xf = x0;
%xf.x = 1;
%xf.z = 3;

% Make mytime allowed to be anywhere within time duration
upperxf = xf;
lowerxf = xf;
upperxf.mytime = maximum_duration;
lowerxf.mytime = minimum_duration;
prog = prog.addStateConstraint(BoundingBoxConstraint(double(lowerxf),double(upperxf)),N);

prog = prog.addInputConstraint(ConstantConstraint(u0),N);

prog = prog.addRunningCost(@cost);
prog = prog.addFinalCost(@finalCost);

tf0 = 2;
traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(upperxf)]));
traj_init.u = ConstantTrajectory(u0);

info=0;
while (info~=1)
  tic
  [xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
  toc
end

if (nargout<1)
  xtraj=xtraj(1:12);
  xtraj = xtraj.setOutputFrame(cfW.manip.getStateFrame);      
  v.playback(xtraj,struct('slider',true));
end

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