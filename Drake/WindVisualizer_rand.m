classdef WindVisualizer_rand < BotVisualizer
% Implements the draw function for the Acrobot 

  methods
    function obj = WindVisualizer_rand(plant, lcmgl)
      % Construct visualizer
      %   AcrobotVisualizer(AcrobotPlant) will take the necessary
      %   parameters from the plant class
      
      %typecheck(plant,'AcrobotPlant');
      obj = obj@BotVisualizer(plant);
      obj.lcmgl = lcmgl;
      %obj.l1 = plant.l1;
      %obj.l2 = plant.l2;
      obj.tvWind_rand_obj = tvWind_rand;
    end
    
    function draw(obj,t,x)
      
      draw@BotVisualizer(obj,t,x)
      
      if ~isempty(obj.lcmgl)
        obj.tvWind_rand_obj.draw(t,obj.lcmgl);
      end
      
    end
  end

  properties
    lcmgl;
    tvWind_rand_obj;

  end
  
end
