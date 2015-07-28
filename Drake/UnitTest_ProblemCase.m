function UnitTest_ProblemCase()

t = 0.5743;
x =  [   0.0100
    0.2749
    0.4824
    0.1154
    0.3394
    0.7101
    0.4604
    0.2797
    0.6332
    0.5895
    0.1137
    0.0417
    0.5743];

u = [0.1555
    0.8425
    0.4356
    0.0251];



rM = QuadWindPlant_numerical;
MATLABans = rM.dynamics_no_grad(t,x,u);

rCpp = QuadWindPlant_numericalCpp;
Cppans = rCpp.dynamics_no_grad(t,x,u);


diffvec = MATLABans - Cppans;
diff = norm(diffvec);

display(diff);


display('MATLAB gives'); display(MATLABans);
display('Cpp gives'); display(Cppans);

xdotlabel = linspace(1,13,13);
figure;
plot(xdotlabel, diffvec, 'rX')
Cppans = rCpp.dynamics_no_grad(t,x,u);


end