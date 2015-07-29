function UnitTests_CrazyFlieWindDynamics

cd /Users/pflomacpro/ProjectWind/Drake/crazyflie-tools-sandbox;

s = rng(123512, 'twister');
for i = 1:10
  

  t = rand(1)*1;
  x12 = rand(12,1);
  x13 = [x12 ; t];
  u = rand(4,1);
  
  rBen = CrazyflieModel;
  Benans = rBen.manip.dynamics(t,x12,u);
  
  rPete = CrazyflieWindModel;
  Peteans = rPete.dynamics_no_grad(t,x13,u);
  Peteans12 = Peteans(1:12);
  
  diffvec = Benans - Peteans12;
  diff = norm(diffvec);
  
  display(diff);
  
  if diff > 1e-12
    display('These inputs are causing trouble');
    display('t: '); display(t);
    display('x13: '); display(x13);
    display('u: '); display(u);
     
    display('Ben gives'); display(Benans);
    display('Pete gives'); display(Peteans);
    
    xdotlabel = linspace(1,12,12);
    figure;
    plot(xdotlabel, diffvec, 'rX')
  end
  
  
  
 
end
rng(s);

end
