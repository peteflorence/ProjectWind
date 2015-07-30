clear cf;
[xtraj, utraj] = runDircol_simpleWindCpp;
cf = Crazyflie();
%ctvlqr = cf.getTvlqr(xtraj,utraj,false);
ctvlqr = cf.getPositionControlTvlqrWind(xtraj,utraj);
cf.runController(ctvlqr);