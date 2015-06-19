function tvWindDraw_FlyingEllipsoid(t)
  
display(t)

ellipsoidcenter = [3 0 1];

windfield = 'flyingellipsoid';


V_0 = 3.5; % m/s guess
c = 0.1; % guess
V = V_0 / (1 + V_0 * c * t);
%V = V_0;

ellipsoidcenter = ellipsoidcenter - [V*t 0 0];
ellipsoidmajor = 0.24;
ellipsoidminor = 0.20;

lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'Windy');
lcmgl.glColor3f(0,1,0);

if strcmp(windfield, 'flyingellipsoid')
  xcenter = ellipsoidcenter(1);
  ycenter = ellipsoidcenter(2);
  zcenter = ellipsoidcenter(3);
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
end


lcmgl.glEnd();
lcmgl.switchBuffers();
pause(0.01)
end



