cd /Users/pflomacpro/ProjectWind/Drake;



for i = 1:50
  
  t = rand(1)*1;
  x = rand(13,1); 
  u = rand(4,1);
  
  rM = QuadWindPlant_numerical;
  MATLABans = rM.dynamics_no_grad(t,x,u);
  
  rCpp = QuadWindPlant_numericalCpp;
  Cppans = rCpp.dynamics_no_grad(t,x,u);
  
  diffvec = MATLABans - Cppans;
  diff = norm(diffvec);
  
  display(diff);
  
  if diff > 1e-3
    display('These inputs are causing trouble');
    display('t: '); display(t);
    display('x: '); display(x);
    display('u: '); display(u);
    
    display('MATLAB gives'); display(MATLABans);
    display('Cpp gives'); display(Cppans);
    
    xdotlabel = linspace(1,13,13);
    plot(xdotlabel, diffvec, 'rX')

    pause;
  end
  
  
  
 
end
