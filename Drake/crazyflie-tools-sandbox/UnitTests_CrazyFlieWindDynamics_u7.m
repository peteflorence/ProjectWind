function UnitTests_CrazyFlieWindDynamics_u7

cd /Users/pflomacpro/ProjectWind/Drake/crazyflie-tools-sandbox;

s = rng(123512, 'twister');
numiter = 20;
r_ddot = zeros(numiter,1);
diff_all = zeros(numiter,1);
for i = 1:numiter
  
  
  t = rand(1);
  x12 = randn(12,1);
  x13 = [x12 ; t];
  u = randn(7,1);
  
  rBen = CrazyflieModel;
  Benans = rBen.dynamics(t,x12,u);
  display('Ben gives'); display(Benans);
  
  rPete = CrazyflieWindModel;
  Peteans = rPete.dynamics(t,x13,u);
  Peteans12 = Peteans(1:12);
  display('Pete gives'); display(Peteans12);
  
  diffvec = Benans - Peteans12;
  diff = norm(diffvec);
  
  diffall(i) = diff;
  
  
  %display('These inputs are causing trouble');
  %display('t: '); display(t);
  %display('x13: '); display(x13);
  %display('u: '); display(u);
  
  xdotlabel = linspace(1,12,12);
  figure;
  plot(xdotlabel, diffvec, 'rX')
  
  
  r_ddot(i) = Benans(10)/Peteans(10);
  
  
  
  
end
display(diffall)
rng(s);
iterlabel = linspace(1,numiter,numiter);
figure
plot(iterlabel,r_ddot);

end
