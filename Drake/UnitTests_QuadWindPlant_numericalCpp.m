cd /Users/pflomacpro/ProjectWind/Drake;


s = rng(123512, 'twister');
for i = 1:50
  

  t = rand(1)*1;
  x = [rand(12,1) ; t]; 
  u = rand(4,1);
  
  rM = QuadWindPlant_numerical;
  MATLABans = rM.dynamics_no_grad(t,x,u);
  
  rCpp = QuadWindPlant_numericalCpp;
  Cppans = rCpp.dynamics_no_grad(t,x,u);
  
  diffvec = MATLABans - Cppans;
  diff = norm(diffvec);
  
  display(diff);
  
  if diff > 1e-12
    display('These inputs are causing trouble');
    display('t: '); display(t);
    display('x: '); display(x);
    display('u: '); display(u);
     
    display('MATLAB gives'); display(MATLABans);
    display('Cpp gives'); display(Cppans);
    
    xdotlabel = linspace(1,13,13);
    figure;
    plot(xdotlabel, diffvec, 'rX')
    Cppans = rCpp.dynamics_no_grad(t,x,u);
  end
  
  
  
 
end
rng(s);
