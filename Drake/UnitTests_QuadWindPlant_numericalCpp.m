cd /Users/pflomacpro/ProjectWind/Drake;



for i = 1:10
  
  t = rand(1)*10;
  x = rand(13,1);
  u = rand(4,1);
  
  rM = QuadWindPlant_numerical;
  MATLABans = rM.dynamics_no_grad(t,x,u);
  
  rCpp = QuadWindPlant_numericalCpp;
  Cppans = rCpp.dynamics_no_grad(t,x,u);
  
  diff = norm(MATLABans - Cppans);
  
  display(diff);
  
  if diff > 1e-3
    pause
  end
 
end
