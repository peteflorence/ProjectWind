classdef tvWind_rand
  % Implements the draw function for the Acrobot
  
  properties
    
    xwind;
    ywind;
    zwind;
    n = 21;
    
    Xwind;
    Ywind;
    Zwind;
    xpos;
    ypos;
    zpos;
    
  end
  
  methods
    
    function obj = tvWind_rand()
      s = rng(123512, 'twister');
      obj.Xwind = 0.1*randn(obj.n,obj.n,obj.n);
      obj.Ywind = zeros(obj.n,obj.n,obj.n);
      obj.Zwind = zeros(obj.n,obj.n,obj.n);
      
      obj.xpos = linspace(-1,1,obj.n);
      obj.ypos = linspace(-1,1,obj.n);
      obj.zpos = linspace(-1,1,obj.n);
 
    end
    
    
    
    function draw(obj, t, lcmgl)
      
      step = 0.25;
      box_l = -1;
      box_u = 1;
      
      obj.Xwind = obj.Xwind + 0.02*randn(obj.n,obj.n,obj.n);
      
      for yi = 1:obj.n
        for xi = 1:obj.n
          for zi = 1:obj.n
            pos = [obj.xpos(xi), obj.ypos(yi), obj.zpos(zi)];
            force = [obj.Xwind(xi,yi,zi), obj.Ywind(xi,yi,zi), obj.Zwind(xi,yi,zi)];
            lcmgl.glColor3f(0,1,0);
            lcmgl.drawVector3d(pos,force);
          end
        end
      end
      
      
      lcmgl.glEnd();
      lcmgl.switchBuffers();
      pause(0.01)
    end
  end
  
  
  
end


