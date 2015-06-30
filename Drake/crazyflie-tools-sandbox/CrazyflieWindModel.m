classdef CrazyflieWindModel < DrakeSystem
  
  properties
    manip;
    pdK;
    nominal_thrust;
    
    ROLL_KP = 3.5*180/pi;
    PITCH_KP = 3.5*180/pi;
    YAW_KP = 3.5*180/pi;
    ROLL_RATE_KP = 70*180/pi;
    PITCH_RATE_KP = 70*180/pi; 
    YAW_RATE_KP = 50*180/pi;
    
    ellipsoidcenter = [0 0 0];
  end
  
  methods
    function obj = CrazyflieWindModel()
      obj = obj@DrakeSystem(13,0,7,13,false,1);
      options.floating = true;
      obj.manip = RigidBodyManipulator('crazyflie.urdf',options);
      obj.nominal_thrust = .25*norm(getMass(obj.manip)*obj.manip.gravity)/obj.manip.force{1}.scale_factor_thrust;
      obj.pdK = [0 obj.PITCH_KP obj.YAW_KP 0 obj.PITCH_RATE_KP obj.YAW_RATE_KP;
                 obj.ROLL_KP 0 -obj.YAW_KP obj.ROLL_RATE_KP 0 -obj.YAW_RATE_KP;
                 0 -obj.PITCH_KP obj.YAW_KP 0 -obj.PITCH_RATE_KP obj.YAW_RATE_KP;
                 -obj.ROLL_KP 0 -obj.YAW_KP -obj.ROLL_RATE_KP 0 -obj.YAW_RATE_KP];
               
      obj = setStateFrame(obj,CoordinateFrame('CrazyFlie13State',13,'x',{'x','y','z','roll','pitch','yaw','xdot','ydot','zdot','rolldot','pitchdot','yawdot','mytime'}));
      obj = obj.setOutputFrame(obj.getStateFrame);      
      
    end
    
    function x0 = getInitialState(obj)
      x0 = zeros(13,1);
    end
    
    function [xdot,dxdot] = dynamics(obj,t,x,u)
      % states: xyz, rpy, dxyz, drpy
      % inputs: rpydesired,omegadesired,thrust
      
      [pqr,dpqr] = rpydot2angularvel(x(4:6),x(10:12));
      [R,dR] = rpy2rotmat(x(4:6));
      dR = [dR,zeros(9,3)];      
      dR = blockwiseTranspose(reshape(full(dR),3,[]),[3,3]);

      pqr = R'*pqr;
      dpqr = -R'*reshape(dR*pqr,3,[]) + R'*dpqr;
      dpqr = [zeros(3,4) dpqr(:,1:3) zeros(3) dpqr(:,4:6) zeros(3,7)];
      
      err = [x(4:6);pqr]-u(1:6);
      derr = [zeros(3,4),eye(3),zeros(3,13);dpqr]-[zeros(6,13),eye(6),zeros(6,1)];
      motorcommands = obj.pdK*err + sqrt(u(7))*10000.0;
      dmotorcommands = obj.pdK*derr + [zeros(4,19),repmat(10000.0/(2*sqrt(u(7))),4,1)];
       
      omegasqu = ((motorcommands)/10000.0).^2;
      domegasqu = 2*repmat(motorcommands/10000.0,1,20).*dmotorcommands/10000.0;
      
      [xdot,dxdot] = obj.dynamics_helper(t,x,omegasqu);    
      
      domegasqu = [domegasqu(:, 1:13) zeros(4,1) domegasqu(:,14:20)];
      dxdot = [dxdot(:,1:14),zeros(13,7)] + dxdot(:,15:18)*domegasqu;
    end
    
    function [xdot,dxdot] = dynamics_helper(obj,t,x,u)
      
      options = struct();
      options.grad_method = 'numerical';
      
      tempfunc = @(t, x, u) obj.dynamics_no_grad(t, x, u);
      
      [xdot, dxdot] = geval(tempfunc, t, x, u, options);
      
       
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
      m = getMass(obj.manip);
      I = obj.manip.body(2).inertia;
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
      
      qdd = [xyz_ddot;rpy_ddot];
      qd = x(7:12);
      xdot = [qd;qdd;1]; % the 1 at the end is for mytime
      
    end
    
    function wind = quadwind(obj,quadpos,mytime,plotme)  
      xquad = quadpos(1);
      yquad = quadpos(2);
      zquad = quadpos(3);   
      windfield = 'flyingsphere';    
      
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
      wind = [xwind;ywind;zwind];
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
    
    
    function y = output(obj,t,x,u)
      y = x;
    end
  end
  
end
