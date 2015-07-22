classdef WindVisualizer < BotVisualizer
% Implements the draw function for the Acrobot 

  methods
    function obj = WindVisualizer(plant, lcmgl)
      % Construct visualizer
      %   AcrobotVisualizer(AcrobotPlant) will take the necessary
      %   parameters from the plant class
      
      %typecheck(plant,'AcrobotPlant');
      obj = obj@BotVisualizer(plant);
      obj.lcmgl = lcmgl;
      %obj.l1 = plant.l1;
      %obj.l2 = plant.l2;
    end
    
    function draw(obj,t,x)
      
      draw@BotVisualizer(obj,t,x)
      if ~isempty(obj.lcmgl)
        tvWindDraw_FlyingSphere_noLogic(t, obj.lcmgl);
      end
      
    end
  end

  properties
    lcmgl;

  end
  
end
