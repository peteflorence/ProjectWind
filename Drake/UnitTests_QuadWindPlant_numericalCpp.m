cd /Users/pflomacpro/ProjectWind/Drake; 
r = QuadWindPlant_numerical; 
r.dynamics_no_grad(1,ones(13,1),ones(4,1))

cd /Users/pflomacpro/ProjectWind/Drake; 
r = QuadWindPlant_numericalCpp; 
r.dynamics_no_grad(1,ones(13,1),ones(4,1))