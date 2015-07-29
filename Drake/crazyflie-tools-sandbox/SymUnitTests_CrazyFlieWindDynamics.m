function SymUnitTests_CrazyFlieWindDynamics

cd /Users/pflomacpro/ProjectWind/Drake/crazyflie-tools-sandbox;




t = rand(1)*1;
x12 = rand(12,1);
x13 = [x12 ; t];
u = rand(4,1);

rBen = CrazyflieModel;
Benans = rBen.manip.dynamics(t,x12,u);
display('Ben gives'); display(Benans);

rPete = CrazyflieWindModel;
Peteans = rPete.dynamics_no_grad(t,x13,u);
Peteans12 = Peteans(1:12);
display('Pete gives'); display(Peteans12);

diffvec = Benans - Peteans12;
diff = norm(diffvec);

display(diff);


end
