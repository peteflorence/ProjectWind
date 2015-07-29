function UnitTests_CrazyFlieWindDynamics_CppvsM

cd /Users/pflomacpro/ProjectWind/Drake/crazyflie-tools-sandbox;

s = rng(123512, 'twister');
numiter = 10;
r_ddot = zeros(numiter,1);
diff_all = zeros(numiter,1);
for i = 1:numiter
  
  
  t = 0;
  x12 = randn(12,1);
  x13 = [x12 ; 0];
  u = [rand(1) rand(1) rand(1) rand(1)]';
  
  rM = CrazyflieWindModel;
  Mans = rM.dynamics_no_grad(t,x13,u);
  Mans = Mans(1:12);
  display('MATLAB gives'); display(Mans);
  
  rCpp = CrazyflieWindModelCpp;
  Cans = rCpp.Cf_dynamics_no_grad(t,x13,u);
  Cans = Cans(1:12);
  display('Cpp gives'); display(Cans);
  
  diffvec = Mans - Cans;
  diff = norm(diffvec);
  
  diffall(i) = diff;
  
  
  %display('These inputs are causing trouble');
  %display('t: '); display(t);
  %display('x13: '); display(x13);
  %display('u: '); display(u);
  
  xdotlabel = linspace(1,12,12);
  figure;
  plot(xdotlabel, diffvec, 'rX')
  
  
  r_ddot(i) = Mans(10)/Cans(10);
  
  
  
  
end
display(diffall)
rng(s);
iterlabel = linspace(1,numiter,numiter);
figure
plot(iterlabel,r_ddot);

end
