function SymUnitTests_CrazyFlieWindDynamics

cd /Users/pflomacpro/ProjectWind/Drake/crazyflie-tools-sandbox;


t = sym('t');
X13 = sym('x',[13,1]);
X12 = X13(1:12);
U   = sym('u',[4,1]);
assume(X13,'real')
assume(X12,'real')
assume(t,'real')
assume(U,'real')

rBen = CrazyflieModel;
Benans = rBen.manip.dynamics(t,X12,U)
display('Ben gives'); display(Benans);

rPete = CrazyflieWindModel;
Peteans = rPete.dynamics_no_grad(t,X13,U);
Peteans12 = Peteans(1:12)
display('Pete gives'); display(Peteans12);

diffvec = Benans - Peteans12


end

