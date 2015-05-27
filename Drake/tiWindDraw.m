function tiWindDraw(windfield)



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
  r.ellipsoidcenter = [3 0 1];
  ellipsoidmajor = 0.24;
  ellipsoidminor = 0.20;
  xcenter = r.ellipsoidcenter(1);
  ycenter = r.ellipsoidcenter(2);
  zcenter = r.ellipsoidcenter(3);
  for yi = (ycenter-ellipsoidmajor):0.05:(ycenter+ellipsoidmajor)
    for xi = (xcenter-ellipsoidminor):0.05:(xcenter+ellipsoidminor)
      for zi = (zcenter-ellipsoidmajor):0.05:(zcenter+ellipsoidmajor)
        ywind = 0;
        zwind = 0;
        if (xi - xcenter)^2/ellipsoidminor^2 + (yi-ycenter)^2/ellipsoidmajor^2 + (zi - zcenter)^2/ellipsoidmajor^2 < 1
          xwind = -0.6/20;
        else
          xwind = 0;
        end
        pos = [xi, yi, zi];
        force = [xwind, ywind, zwind];
        lcmgl.drawVector3d(pos,force);
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
end
