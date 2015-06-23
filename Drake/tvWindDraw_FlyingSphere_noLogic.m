function tvWindDraw_FlyingSphere_noLogic(t)
  
display(t)

ellipsoidcenter = [2 0 1];

windfield = 'flyingsphere';

V_0 = 3.5; % m/s guess
c = 0.1; % guess
V = V_0 / (1 + V_0 * c * t);
%V = V_0;

ellipsoidcenter = ellipsoidcenter - [V*t 0 0];

sphereRadius = 0.3;
boundary = 0.1;
extSphere = sphereRadius + boundary;

lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'Windy');
lcmgl.glColor3f(0,1,0);

step = 0.10;
nomwind = -3.0;

if strcmp(windfield, 'flyingsphere')
  xcenter = ellipsoidcenter(1);
  ycenter = ellipsoidcenter(2);
  zcenter = ellipsoidcenter(3);
  
  display(ellipsoidcenter)
  
  for yi = (ycenter-extSphere):step:(ycenter+extSphere)
    yidif = yi - ycenter;          % translate
    for xi = (xcenter-extSphere):step:(xcenter+extSphere)
      xidif = xi - xcenter;        % translate
      for zi = (zcenter-extSphere):step:(zcenter+extSphere)
        zidif = zi - zcenter;      % translate
        
        
        xwind = 0;
        ywind = 0;
        zwind = 0;
        

        % cart2sph
        %[azimuth,elevation,r] = cart2sph(xidif,yidif,zidif);
        

        scale = nomwind;
        
        reversed = -1;
        
        a = sqrt(xidif^2 + yidif^2 + zidif^2);
        
        slope = 10;
        
        xwind = scale * (tanh(reversed * ( a - sphereRadius) * slope ) +1) / 2;
        
        if abs(xwind) > 0.1
          pos = [xi, yi, zi];
          force = [xwind, ywind, zwind];
          lcmgl.drawVector3d(pos,force/20);
        end
      end
      
    end
  end
end


lcmgl.glEnd();
lcmgl.switchBuffers();
pause(0.01)
end



