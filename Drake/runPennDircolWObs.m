function [utraj,xtraj,prog,r] = runPennDircolWObs

%plant = 'russ';
plant = 'penn';

if plant == 'russ'
  r = Quadrotor();
  v = constructVisualizer(r);
  disp('using russ rotor!')
elseif plant == 'penn'
  r_temp = Quadrotor();
  %r_temp = addOcean(r_temp, [.8,.45,1.25], [.20;2.5], pi/4);
  v = constructVisualizer(r_temp);
  r = QuadWindPlant_noWindGradients(); % Quadrotor constructor

  disp('using quad plant in wind based on penn plant!!')
end


N = 21;
minimum_duration = .1;
maximum_duration = 3;
prog = DircolTrajectoryOptimization(r,N,[minimum_duration maximum_duration]);

x0 = Point(getStateFrame(r));  % initial conditions: all-zeros

if plant == 'russ'
  x0.base_z = 1; % lift the quad off the ground
elseif plant == 'penn'
  x0.z = 1; % lift the quad off the ground
end

v.draw(0,double(x0));
prog = addPlanVisualizer(r,prog);
u0 = double(nominalThrust(r));
prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1); % DirectTrajectoryOptimization method
[ground,world,Ncell] = createGroundBoundary(r,N);
prog = prog.addStateConstraint(BoundingBoxConstraint(double(ground),double(world)),Ncell);
prog = prog.addInputConstraint(ConstantConstraint(u0),1); % DirectTrajectoryOptimization method

xf = x0;                       % final conditions: translated in x
upperxf = x0;
lowerxf = x0;

if plant == 'russ'
  xf.base_x = 2;                 % translate x by 2
elseif plant == 'penn'
  upperxf.x = 5;                 % translate x
  upperxf.z = 1;                 % translate z
  upperxf.y = 0;                 % translate x
  upperxf.mytime = maximum_duration;
  
  lowerxf.x = 5;                 % translate x
  lowerxf.z = 1;                 % translate z
  lowerxf.y = 0;                 % translate x
  lowerxf.mytime = minimum_duration;
  
end



prog = prog.addStateConstraint(BoundingBoxConstraint(double(lowerxf),double(upperxf)),N);
% Constant Constraint is simpler for a time-invarying problem
% prog = prog.addStateConstraint(ConstantConstraint(double(xf)),N); % DirectTrajectoryOptimization method
prog = prog.addInputConstraint(ConstantConstraint(u0),N); % DirectTrajectoryOptimization method

prog = prog.addRunningCost(@cost); % note: DirCol not Direct!  DircolTrajectoryOptimization method
prog = prog.addFinalCost(@finalCost); % DirectTrajectoryOptimization method

tf0 = 2;                      % initial guess at duration
traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)])); % traj.init.x is a PPTrajectory < Trajectory < DrakeSystem
traj_init.u = ConstantTrajectory(u0); % traj_init.u is a ConstantTrajectory < Trajectory < DrakeSystem

info=0;
while (info~=1)
  tic
  snseti ('Verify level', 3);
  [xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init); % DirectTrajectoryOptimization method
  toc
end










% CREATE TVLQR

tic;
x0 = xtraj.eval(0);
tf = utraj.tspan(2);
% Q = 10*eye(13);
Q = 10 * [eye(3) zeros(3,10) ; zeros(10,3) zeros(10)];
R = eye(4);
Qf = 10*eye(13);
disp('Computing stabilizing controller with TVLQR...');
ltvsys = tvlqr(r,xtraj,utraj,Q,R,Qf);



% CREATE FEEDBACK CONTROLLER
% For no Gaussian noise (simulating on same plant):
sys = feedback(r,ltvsys);

% For Gaussian noise (simulating on different plant):
%sys = feedback(r2,ltvsys);

toc;
disp('done!');

% Cascade if desired
%tic;
%sys = cascade(utraj, r);
%toc;

% Simulate the result
tic;
disp('Simulating the system...');
%xtraj_sim = simulate(sys,[0 tf],x0);
%xtraj_sim = simulate(sys,[0 tf],x0);
%toc;
%disp('done!');


% Draw the TVLQR result
%xtraj_sim = xtraj_sim(1:12);
%xtraj_sim = xtraj_sim.setOutputFrame(r_temp.getStateFrame());
%v.playback(xtraj_sim, struct('slider', true));


% % Draw the original result
% if (nargout<1)
%    xtraj = xtraj(1:12);
%    xtraj = xtraj.setOutputFrame(r_temp.getStateFrame());
%    v2.playback(xtraj,struct('slider',true));
% end




% BEN'S CASCADE CODE

%utraj = setOutputFrame(utraj,getInputFrame(r_temp));
%sys = cascade(utraj,r);
%systraj = sys.simulate([0 utraj.tspan(2)],xtraj.eval(0));
%v.playback(systraj,struct('slider',true));






% PLOT WIND
%[winddontcare,dquadinwinddontcare] = quadwind(r,[0,0,0],0,1);

%xquad = quadpos(1);
%yquad = quadpos(2);
%zquad = quadpos(3);

windfield = 'zero';
%windfield = 'constant';
%windfield = 'linear';
%windfield = 'quadratic';
%windfield = 'sqrt';
%windfield = 'exp';
%windfield = 'difftailhead';
%windfield = 'thermals';
%windfield = 'tvsin';
%windfield = 'tlinear';
%windfield = 'flyingellipsoid';
r.ellipsoidcenter = [3 0 1];
ellipsoidmajor = 0.24;
ellipsoidminor = 0.20;




lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'Windy');
lcmgl.glColor3f(0,1,0);

if strcmp(windfield, 'difftailhead')
  for yi = -5:0.5:5
    xwind = 10*sin(yi);
    ywind = 0;
    zwind = 0;
    pos = [0, yi, 0];
    force = [xwind, ywind, zwind];
    %lcmgl.drawVector3d([0,0,0],[1,1,1]);
    lcmgl.drawVector3d(pos,force);
  end
elseif strcmp(windfield, 'thermals')
  for yi = -5:0.2:5
    for xi = 1:0.2:10
    xwind = 0;
    ywind = 0;
    if (xi - 3)^2 + yi^2 < 1
      zwind = 3;
    else
      zwind = 0;
    end
    pos = [xi, yi, 0];
    force = [xwind, ywind, zwind];
    lcmgl.drawVector3d(pos,force);
    end
  end
elseif strcmp(windfield, 'flyingellipsoid')
  xcenter = r.ellipsoidcenter(1);
  ycenter = r.ellipsoidcenter(2);
  zcenter = r.ellipsoidcenter(3);  
  for yi = (ycenter-ellipsoidmajor):0.05:(ycenter+ellipsoidmajor)
    for xi = (xcenter-ellipsoidminor):0.05:(xcenter+ellipsoidminor)
      for zi = (zcenter-ellipsoidmajor):0.05:(zcenter+ellipsoidmajor)
        ywind = 0;
        zwind = 0;
        if (xi - xcenter)^2/ellipsoidminor^2 + (yi-ycenter)^2/ellipsoidmajor^2 + (zi - zcenter)^2/ellipsoidmajor^2 < 1
          xwind = -0.6;
        else
          xwind = 0;
        end
        pos = [xi, yi, zi];
        force = [xwind, ywind, zwind];
        lcmgl.drawVector3d(pos,force/20);
      end
      
    end
  end
else
  for xi = 1:10
    %for yi = 1:10
    for zi = 1:10
      
      xwind = 0;
      
      if strcmp(windfield, 'zero')
        ywind = 0;
      elseif strcmp(windfield, 'constant')
        ywind = 2;
      elseif strcmp(windfield, 'linear')
        ywind = zi;
      elseif strcmp(windfield, 'quadratic')
        ywind = zi^2;
      elseif strcmp(windfield, 'sqrt')
        ywind = (abs(zquad))^(1/2);
      elseif strcmp(windfield, 'exp')
        a = 1;
        C=20/exp(6.5);
        b=-1;
        d = 0;
        %z  = b + C*exp(a*ydotdot);
        ywind = 1/a*log((zi-b)/C) - d;
      elseif strcmp(windfield, 'tvsin')
        ywind = -10*sin(10*mytime);
      elseif strcmp(windfield, 'tlinear')
        ywind = -5 - mytime;
      else
        disp('Please specify which kind of wind field!')
      end
      
      zwind = 0;
      
      pos = [xi, 0, zi];
      force = [xwind, ywind, zwind];
      %lcmgl.drawVector3d([0,0,0],[1,1,1]);
      lcmgl.drawVector3d(pos,force);

    end
    
  end
end

lcmgl.glEnd();
lcmgl.switchBuffers();

%lcmgl.glColor3f(0, 0, 1);
%lcmgl.plot3(x(1,1:2)+1,x(2,1:2),x(3,1:2));



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



