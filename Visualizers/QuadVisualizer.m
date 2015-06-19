classdef QuadVisualizer < Visualizer
% Implements the draw function for the Penn Quadrotor

  properties
      angleRep = 'euler';
      forest = 0;
  end

  methods
    function obj = QuadVisualizer(p,angleRep)
      obj = obj@Visualizer(p.getOutputFrame);
      obj.playback_speed = 1;
      % obj.display_dt = 0.1;
      if (nargin > 1)
          obj.angleRep = angleRep;
      else
          obj.angleRep = 'euler';
      end
    end
    
    function [h1,h2,h3,h4,h5,h6] = draw(obj,t,x)
        persistent hFig;

        if (isempty(hFig))
            hFig = sfigure(25);
            set(hFig,'DoubleBuffer', 'on');
        end
        
        hold on;
              
        %Airplane definition
        cross_scale=0.75/3;
        air_cross_y=[0.15;0.15;1;1;0.15;0.15;-0.15;-0.15;-1;-1;-0.15;-0.15;0.15];
        air_cross_x=[1;0.15;0.15;-0.15;-0.15;-1;-1;-0.15;-0.15;0.15;0.15;1;1];
        air_cross = [air_cross_x,air_cross_y];
        air_cross=cross_scale*air_cross;
        x_coll_cross = [air_cross(:,1),air_cross(:,2),zeros(size(air_cross,1),1)];
       

        air_body_y=[0.25;0.25;-0.25;-0.25];
        air_body_x=[0.25;-0.25;-0.25;0.25];
        air_body=[air_body_x,air_body_y];
        air_body=cross_scale*air_body;
        x_coll_body = [air_body(:,1),air_body(:,2),zeros(size(air_body,1),1)];
        
        x_coll_rotor1 = plotEllipse(eye(2),0.25,[1;0],'r');
        x_coll_rotor1 = cross_scale*[x_coll_rotor1',0.05*ones(size(x_coll_rotor1,2),1)];
        
        x_coll_rotor2 = plotEllipse(eye(2),0.25,[0;1],'r');
        x_coll_rotor2 = cross_scale*[x_coll_rotor2',0.05*ones(size(x_coll_rotor2,2),1)];
        
        x_coll_rotor3 = plotEllipse(eye(2),0.25,[-1;0],'r');
        x_coll_rotor3 = cross_scale*[x_coll_rotor3',0.05*ones(size(x_coll_rotor3,2),1)];
       
        x_coll_rotor4 = plotEllipse(eye(2),0.25,[0;-1],'r');
        x_coll_rotor4 = cross_scale*[x_coll_rotor4',0.05*ones(size(x_coll_rotor4,2),1)];
        
        X_LOW = -2;
        X_HIGH = 2;
        Y_LOW = -2;
        Y_HIGH = 2;
        Z_LOW = -1;
        Z_HIGH = 1;

        curr_window = [X_LOW,X_HIGH,Y_LOW,Y_HIGH,Z_LOW,Z_HIGH];

        axis_size = curr_window;

      %Plot the airplane
      if(~isempty(x))
          plane_color=[0, 0, 0];
          [x_cross,y_cross,z_cross]=plane_coll_points(x,x_coll_cross,obj.angleRep);
          [x_body,y_body,z_body]=plane_coll_points(x,x_coll_body,obj.angleRep);
          [x_rotor1,y_rotor1,z_rotor1]=plane_coll_points(x,x_coll_rotor1,obj.angleRep);
          [x_rotor2,y_rotor2,z_rotor2]=plane_coll_points(x,x_coll_rotor2,obj.angleRep);
          [x_rotor3,y_rotor3,z_rotor3]=plane_coll_points(x,x_coll_rotor3,obj.angleRep);
          [x_rotor4,y_rotor4,z_rotor4]=plane_coll_points(x,x_coll_rotor4,obj.angleRep);
          h1 = fill3(x_cross+x(1),y_cross+x(2),z_cross+x(3),plane_color);
          %hold on
          h2 = fill3(x_body+x(1),y_body+x(2),z_body+x(3),plane_color);
          h3 = fill3(x_rotor1+x(1),y_rotor1+x(2),z_rotor1+x(3),[1 0 0]);
          h4 = fill3(x_rotor2+x(1),y_rotor2+x(2),z_rotor2+x(3),[1 0 0]);
          h5 = fill3(x_rotor3+x(1),y_rotor3+x(2),z_rotor3+x(3),[1 0 0]);
          h6 = fill3(x_rotor4+x(1),y_rotor4+x(2),z_rotor4+x(3),[1 0 0]);
          %hold off
      end
      
%       %Plot the forest
%       if(size(obj.forest) > 1)
%           for j = 1:size(obj.forest,2)
%                line([obj.forest(1,j),obj.forest(1,j)],[obj.forest(2,j),obj.forest(2,j)],[-2,2],'Color','b','LineWidth',5);
%           end
%       end
          
      
      title(strcat('t = ', num2str(t,'%.5f')));
      axis(axis_size);
      view([-5,5,5])
      %view([0,0,10])
      set(gca,'DataAspectRatio',[1 1 1]);
      grid on;
      xlabel('x');
      ylabel('y');
      zlabel('z');
    end    
  end
end


function [x_out,y_out,z_out]=plane_coll_points(x,x_coll,angleRep)

if strcmp(angleRep,'euler')
    angs = x(4:6);
    Rot = RPYtoRot_ZXY(angs(1),angs(2),angs(3)); 
    points = Rot*x_coll';
    points = points';
%     q = RotToQuat(Rot);
%     keyboard;
    %points = qrot3d(x_coll,q(2:4),q(1)); %[q(end),q(1:3)]);
elseif strcmp(angleRep,'quaternion')
    error('Quaterion is not supported yet');
    %q = x(7:10);
    %points = qrot3d(x_coll,q(2:4),q(1)); %[q(end),q(1:3)]);
else
    error('This angle representation is not supported');
end
    

x_out = points(:,1);
y_out = points(:,2);
z_out = points(:,3);

end

