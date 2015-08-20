function [xtraj,utraj,prog] = runDircolBox()

cf = CrazyflieModel();

N = 60; 
minimum_duration = 2;
maximum_duration = 20;
prog = DircolTrajectoryOptimization(cf,N,[minimum_duration maximum_duration]);  

x0 = Point(getStateFrame(cf.manip));
x0.base_x = 0.5;
x0.base_y = 1.90;
x0.base_z = 0.5;
u0 = [0 0 0 0 0 0 cf.nominal_thrust]';

prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1);
prog = prog.addInputConstraint(ConstantConstraint(u0),1);
v = constructVisualizer(cf.manip);
v.draw(0,double(x0));

x1 = x0;
x1.base_x = x1.base_x - 1;
prog = prog.addStateConstraint(ConstantConstraint(double(x1)),round(N/4));

x2 = x1;
x2.base_y = x2.base_y - 1;
prog = prog.addStateConstraint(ConstantConstraint(double(x2)),round(N/2));

x3 = x2;
x3.base_x = x3.base_x + 1;
prog = prog.addStateConstraint(ConstantConstraint(double(x3)),round(3*N/4));

xf = x0;
prog = prog.addStateConstraint(ConstantConstraint(double(xf)),N);
prog = prog.addInputConstraint(ConstantConstraint(u0),N);


prog = addPlanVisualizer(cf,prog);


prog = prog.addRunningCost(@cost);
prog = prog.addFinalCost(@finalCost);

tf0 = maximum_duration;
traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));
traj_init.u = ConstantTrajectory(u0);

info=0;
while (info~=1)
  tic
  [xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
  toc
end

if (nargout<1)
  xtraj = xtraj.setOutputFrame(cf.manip.getStateFrame);      
  v.playback(xtraj,struct('slider',true));
end

end

function [g,dg] = cost(dt,x,u)
  R = eye(7);%[eye(4) zeros(4,3); zeros(3,4) 1e-9*ones(3,3)];
  g = u'*R*u;
  dg = [0,zeros(1,size(x,1)),2*u'*R];
end

function [h,dh] = finalCost(t,x)
  h = t;
  dh = [1,zeros(1,size(x,1))];
end