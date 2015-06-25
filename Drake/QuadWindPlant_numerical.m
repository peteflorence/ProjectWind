classdef QuadWindPlant_numerical < DrakeSystem
  
  % Modified from D. Mellinger, N. Michael, and V. Kumar,
  % "Trajectory generation and control for precise aggressive maneuvers with quadrotors",
  %  In Proceedings of the 12th International Symposium on Experimental Robotics (ISER 2010), 2010.
  
  % Adapted by Pete Florence, 2015 to handle simple vortex rings
  
  methods
    function obj = QuadWindPlant_numerical()
      
      obj = obj@DrakeSystem(13,0,4,13,false,1);
      
      obj = setStateFrame(obj,CoordinateFrame('QuadState',13,'x',{'x','y','z','roll','pitch','yaw','xdot','ydot','zdot','rolldot','pitchdot','yawdot','mytime'}));
      obj = obj.setOutputFrame(obj.getStateFrame);
      
    end
    
    function m = getMass(obj)
      m = obj.m;
    end
    
    function I = getInertia(obj)
      I = obj.I;
    end
    
    function u0 = nominalThrust(obj)
      % each propellor commands -mg/4
      u0 = Point(getInputFrame(obj),getMass(obj)*norm(obj.gravity)*ones(4,1)/4);
    end
    
    %taken from rigid body manipulator
    function g = getGravity(obj)
      g = r.gravity;
    end
    
    
    
    function [xdot,df] = dynamics(obj,t,x,u)
      
      options = struct();
      options.grad_method = 'numerical';
      
      tempfunc = @(t, x, u) obj.dynamics_no_grad(t, x, u);
      
      [xdot, df] = geval(tempfunc, t, x, u, options);
      
       
    end
    
    
    
    function xdot = dynamics_no_grad(obj,t,x,u)
      % States
      % x
      % y
      % z
      % phi (roll)
      % theta (pitch)
      % psi (yaw)
      % xdot
      % ydot
      % zdot
      % phidot
      % thetadot
      % psidot
      % time
      
      
      % Parameters
      m = obj.m;
      I = obj.I;
      invI = diag(1./[0.0023,0.0023,0.004]);
      g = 9.81;
      L = 0.1750;
      
      % states
      phi = x(4);
      theta = x(5);
      psi = x(6);
      
      phidot = x(10);
      thetadot = x(11);
      psidot = x(12);
      
      w1 = u(1);
      w2 = u(2);
      w3 = u(3);
      w4 = u(4);
      
      % Rotation matrix from body to world frames
      R = rpy2rotmat([phi;theta;psi]);
      
      kf = 1; % 6.11*10^-8;
      
      F1 = kf*w1;
      F2 = kf*w2;
      F3 = kf*w3;
      F4 = kf*w4;
      
      km = 0.0245;
      
      M1 = km*w1;
      M2 = km*w2;
      M3 = km*w3;
      M4 = km*w4;
      
      xquad = x(1);
      yquad = x(2);
      zquad = x(3);
      quadpos = [xquad;yquad;zquad];
      
      
      
      
      
      
      
      
      
      
      windout = obj.quadwind(quadpos,x(13),1); % pass mytime to quadwind. % last arument is plot option
      
      
      
      
      
      
      
      
      %if (nargout>1)
      %  df = df + dquadinwind;
      %end
      
      xyz_ddot = (1/m)*([0;0;-m*g] + R*[0;0;F1+F2+F3+F4] + windout); % call to wind field in dynamics
      
      pqr = rpydot2angularvel([phi;theta;psi],[phidot;thetadot;psidot]);
      pqr = R'*pqr;
      
      pqr_dot = invI*([L*(F2-F4);L*(F3-F1);(M1-M2+M3-M4)] - cross(pqr,I*pqr));
      
      % Now, convert pqr_dot to rpy_ddot
      [Phi, dPhi] = angularvel2rpydotMatrix([phi;theta;psi]);
      
      Rdot =  [ 0, sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta),   cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta); ...
        0, cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi), - cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta); ...
        0,                              cos(phi)*cos(theta),                               -cos(theta)*sin(phi)]*phidot + ...
        [ -cos(psi)*sin(theta), cos(psi)*cos(theta)*sin(phi), cos(phi)*cos(psi)*cos(theta); ...
        -sin(psi)*sin(theta), cos(theta)*sin(phi)*sin(psi), cos(phi)*cos(theta)*sin(psi); ...
        -cos(theta),         -sin(phi)*sin(theta),         -cos(phi)*sin(theta)]*thetadot + ...
        [ -cos(theta)*sin(psi), - cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta), cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta); ...
        cos(psi)*cos(theta),   cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta); ...
        0,                                                  0,                                                0]*psidot;
      
      rpy_ddot = Phi*R*pqr_dot + reshape((dPhi*[phidot;thetadot;psidot]),3,3)*R*pqr + ...
        Phi*Rdot*pqr;
      
      % xdot = [x(7:12);xyz_ddot;rpy_ddot];
      
      qdd = [xyz_ddot;rpy_ddot];
      qd = x(7:12);
      xdot = [qd;qdd;1]; % the 1 at the end is for mytime
      
    end
    
    function wind = quadwind(obj,quadpos,mytime,plotme)
      
      xquad = quadpos(1);
      yquad = quadpos(2);
      zquad = quadpos(3);
      
      windfield = 'flyingsphere';
      
      xwind = 0;
      ywind = 0;
      zwind = 0;
      if strcmp(windfield, 'difftailhead')
        xwind = 10*sin(yquad);
      end
      
      if strcmp(windfield, 'thermals')
        if (xquad - 3)^2 + yquad^2 < 1
          zwind = 3;
        else
          zwind = 0;
        end
      end
      
      if strcmp(windfield, 'flyingellipsoid')
        V_0 = 3.5; % m/s guess
        c = 0.1; % guess
        V = V_0 / (1 + V_0 * c * mytime);
        
        obj.ellipsoidcenter = obj.ellipsoidcenter - [V*obj.tstep 0 0];
        xcenter = obj.ellipsoidcenter(1);
        ycenter = obj.ellipsoidcenter(2);
        zcenter = obj.ellipsoidcenter(3);
        
        ellipsoidmajor = 0.24;
        ellipsoidminor = 0.20;
        
        ywind = 0;
        zwind = 0;
        if (xquad - xcenter)^2/ellipsoidminor^2 + (yquad-ycenter)^2/ellipsoidmajor^2 + (zquad - zcenter)^2/ellipsoidmajor^2 < 1
          xwind = -100;
        else
          xwind = 0;
        end
      end
      
      if strcmp(windfield, 'flyingspherelogic')
        V_0 = 3.5; % m/s guess
        c = 0.1; % guess
        V = V_0 / (1 + V_0 * c * mytime);
        
        obj.ellipsoidcenter = [3 0 1];
        obj.ellipsoidcenter = obj.ellipsoidcenter - [V*mytime 0 0];
        xcenter = obj.ellipsoidcenter(1);
        ycenter = obj.ellipsoidcenter(2);
        zcenter = obj.ellipsoidcenter(3);
        
        sphereradius = 0.30;
        boundary = 0.01;
        extsphere = sphereradius + boundary;
        
        nomwind = -1.0;
        
        xwind = 0;
        ywind = 0;
        zwind = 0;
        if (xquad - xcenter)^2/sphereradius^2 + (yquad-ycenter)^2/sphereradius^2 + (zquad - zcenter)^2/sphereradius^2 < 1
          xwind = nomwind;
        elseif (xquad - xcenter)^2/extsphere^2 + (yquad-ycenter)^2/extsphere^2 + (zquad - zcenter)^2/extsphere^2 < 1
          display('found a boundary point')
          
          % hypertan wants to go from -1 to 1
          % this f(x) goes from 0 to 3, as x goes from -1 to 1
          
          % find distance to edge of sphere
          fromcenter = sqrt( (xquad - xcenter)^2 + (yquad-ycenter)^2 + (zquad - zcenter)^2 );
          distance = fromcenter - sphereradius;
          
          
          % normalize that distance
          normdistance = distance / boundary;
          % make it go from -1 to 1
          hypertandistance = normdistance * 2 - 1;
          
          % in this setup, -1 is right next to the sphere, and 1 is out in
          % nowhere
          
          reversed = -1;
          xwind = nomwind * (tanh(reversed * hypertandistance * 10)+1)/2;
          
        else
          xwind = 0;
        end
        
      end
      
      
      if strcmp(windfield, 'flyingsphere')
        V_0 = 3.5; % m/s guess
        c = 0.1; % guess
        V = V_0 / (1 + V_0 * c * mytime);
        
        obj.ellipsoidcenter = [2 0 1];
        obj.ellipsoidcenter = obj.ellipsoidcenter - [V*mytime 0 0];
        xcenter = obj.ellipsoidcenter(1);
        ycenter = obj.ellipsoidcenter(2);
        zcenter = obj.ellipsoidcenter(3);
        
        sphereRadius = 0.30;
        
        nomwind = -5.0;
        
        xwind = 0;
        ywind = 0;
        zwind = 0;
        
        xidif = xquad - xcenter;
        yidif = yquad - ycenter;
        zidif = zquad - zcenter; 
        
        scale = nomwind;
        
        reversed = -1;
        
        a = sqrt(xidif^2 + yidif^2 + zidif^2);
        
        slope = 10;
        
        xwind = scale * (tanh(reversed * ( a - sphereRadius) * slope ) +1) / 2;
        
      end
      
      
      
      if strcmp(windfield, 'zero')
        ywind = 0;
      elseif strcmp(windfield, 'constant')
        ywind = 2;
      elseif strcmp(windfield, 'linear')
        ywind = zquad;
      elseif strcmp(windfield, 'quadratic')
        ywind = zquad^2;
      elseif strcmp(windfield, 'sqrt')
        ywind = (abs(zquad))^(1/2);
      elseif strcmp(windfield, 'exp')
        a = 1;
        C=20/exp(6.5);
        b=-1;
        d = 0;
        %z  = b + C*exp(a*ydotdot);
        ywind = 1/a*log((zquad-b)/C) - d;
      elseif strcmp(windfield, 'difftailhead')
        ywind = 0;
      elseif strcmp(windfield, 'tvsin')
        ywind = -10*sin(10*mytime);
      elseif strcmp(windfield, 'tlinear')
        ywind = -5 - mytime;
      else
        %disp('Please specify which kind of wind field!')
      end
      
      
      
      wind = [xwind;ywind;zwind];
      
      
      dquadinwind = sparse(13,18);
      
      %       if strcmp(windfield, 'zero')
      %         ;
      %       elseif strcmp(windfield, 'flyingellipsoid')
      %         ;
      %       elseif strcmp(windfield, 'constant')
      %         ;
      %       elseif strcmp(windfield, 'thermals')
      %         ;
      %       elseif strcmp(windfield, 'linear')
      %         dquadinwind(8,4) = 1/obj.m;
      %       elseif strcmp(windfield, 'quadratic')
      %         dquadinwind(8,4) = 2*zquad/obj.m;
      %       elseif strcmp(windfield, 'sqrt')
      %         ywind = 1/2*(abs(zquad))^(-1/2);
      %       elseif strcmp(windfield, 'exp')
      %         dquadinwind(8,4) = 1/a*1/((zquad-b)/C)/obj.m;
      %       elseif strcmp(windfield, 'difftailhead')
      %         dquadinwind(7,3) = 10*cos(yquad)/obj.m;
      %       elseif strcmp(windfield, 'tvsin')
      %         dquadinwind(8,1) = -10*cos(10*mytime)/obj.m;
      %       elseif strcmp(windfield, 'tlinear')
      %         dquadinwind(8,1) = -1/obj.m;
      %         %else
      %         %  disp('Please specify which kind of wind field!')
      %       end
      
      
      % I think all this code can go... I've figured out drawing elsewhere
      
      %       lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'Windy');
      %       lcmgl.glColor3f(0,1,0);
      %
      %       if plotme == 1 && strcmp(windfield, 'flyingellipsoid')
      %         xcenter = obj.ellipsoidcenter(1);
      %         ycenter = obj.ellipsoidcenter(2);
      %         zcenter = obj.ellipsoidcenter(3);
      %         for yi = (ycenter-ellipsoidmajor):0.05:(ycenter+ellipsoidmajor)
      %           for xi = (xcenter-ellipsoidminor):0.05:(xcenter+ellipsoidminor)
      %             for zi = (zcenter-ellipsoidmajor):0.05:(zcenter+ellipsoidmajor)
      %               ywind = 0;
      %               zwind = 0;
      %               if (xi - xcenter)^2/ellipsoidminor^2 + (yi-ycenter)^2/ellipsoidmajor^2 + (zi - zcenter)^2/ellipsoidmajor^2 < 1
      %                 xwind = -0.6;
      %               else
      %                 xwind = 0;
      %               end
      %               pos = [xi, yi, zi];
      %               force = [xwind, ywind, zwind];
      %               lcmgl.drawVector3d(pos,force/20);
      %             end
      %
      %           end
      %         end
      %       elseif plotme == 1
      %         for xi = 1:10
      %           %for yi = 1:10
      %           for zi = 1:10
      %             pos = [xi, 0, zi];
      %             force = [xwind, ywind, zwind];
      %             %lcmgl.drawVector3d([0,0,0],[1,1,1]);
      %             lcmgl.drawVector3d(pos,force);
      %           end
      %
      %         end
      %
      %
      %
      %       end
      %       lcmgl.glEnd();
      %       lcmgl.switchBuffers;
      
      
    end
    
    
    function y = output(obj,t,x,u)
      y = x;
    end
    
    function x = getInitialState(obj)
      x = zeros(13,1);
    end
    
    function obj = addTrees(obj,number_of_obstacles)
      % Adds a random forest of trees
      if nargin<2, number_of_obstacles = 5*(randi(5)+2); end
      for i=1:number_of_obstacles
        % Populates an area of the forest
        xy = [20,0;0,12]*(rand(2,1) - [0.5;0]);
        % Creates a clear path through the middle of the forest
        while(norm(xy)<1 || (xy(1,1)<=1.5 && xy(1,1)>=-1.5)), xy = randn(2,1); end
        height = 1+rand;
        width_param = rand(1,2);
        yaw = randn;
        obj = obj.addTree([width_param height],xy,yaw);
      end
      obj = compile(obj);
    end
    
    function obj = addTree(obj, lwh, xy, yaw)
      % Adds a single tree with specified length width height, xy
      % location, and yaw orientation.
      height = lwh(1,3);
      width_param = lwh(1,1:2);
      treeTrunk = RigidBodyBox([.2+.8*width_param height],...
        [xy;height/2],[0;0;yaw]);
      treeTrunk.c = [83,53,10]/255;  % brown
      obj = addGeometryToBody(obj,'world',treeTrunk);
      treeLeaves = RigidBodyBox(1.5*[.2+.8*width_param height/4],...
        [xy;height + height/8],[0;0;yaw]);
      treeLeaves.c = [0,0.7,0];  % green
      obj = addGeometryToBody(obj,'world',treeLeaves);
      obj = compile(obj);
    end
    
    function traj_opt = addPlanVisualizer(obj,traj_opt)
      % spew out an lcmgl visualization of the trajectory.  intended to be
      % used as a callback (fake objective) in the direct trajectory
      % optimization classes
      
      if ~checkDependency('lcmgl')
        warning('lcmgl dependency is missing.  skipping visualization');
        return;
      end
      lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'QuadrotorPlan');
      
      typecheck(traj_opt,'DirectTrajectoryOptimization');
      

      traj_opt = traj_opt.addDisplayFunction(@(x)visualizePlan(x,lcmgl),traj_opt.x_inds(1:3,:));
      %traj_opt = traj_opt.addDisplayFunction(@(x)assert(fscanf(fopen('abort.txt', 'r'), '%d') == 0, 'Abort from file.'));
      %fclose('all');
      
      
      function visualizePlan(x,lcmgl)
        lcmgl.glColor3f(1, 0, 0);
        lcmgl.glPointSize(3);
        lcmgl.points(x(1,:),x(2,:),x(3,:));
        lcmgl.glColor3f(1, .5, 0);
        lcmgl.plot3(x(1,:),x(2,:),x(3,:));
        lcmgl.switchBuffers;

        
      end
      
      
      
    end

  end
  
  
  
  
  properties
    m = .5;
    I = diag([0.0023,0.0023,0.004]);
    
    % defining gravity here (too simple?)
    gravity = [0; 0; -9.8100]
    
    
    windfield = 'zero'; % zero windfield by default
    
    ellipsoidcenter = [0 0 0];
    tstep = 0.01;
  end
  
end



