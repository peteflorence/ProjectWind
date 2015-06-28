% The time intervals to use
% Use plotlog to identify those
T = [
1 9.637 10.1;
1 10.1 10.7;
2 10.43 10.86;
3 11.04 11.46;
4 15.27 15.85;
5 9.253 10.54;
6 10.97 11.39;
7 20.13 20.5;
7 20.5 20.97;
8 9.981 10.47;
9 9.109 9.883;
% 1 16.08 16.81;
% 2 14.73 15.52;
% 3 12.01 12.95;
% 4 11.23 12.21;
% 5 14.48 14.95;
% 6 16.17 16.71;
% 7 11.73 12.12;
% 8 17.19 17.72;
% 9 12.94 14.32;
];

for i=1:size(T,1)
  t0 = T(i,2);
  tf = T(i,3);
  rawdata = load([num2str(T(i,1)) '.mat']);

  %pos = [rawdata.crazyflie_state_estimate(:,2:4),rawdata.crazyflie_state_estimate(:,11:13),rawdata.crazyflie_state_estimate(:,15)];
  pos = [rawdata.crazyflie_state_estimate(:,2:4),rawdata.crazyflie_state_estimate(:,5:7),rawdata.crazyflie_state_estimate(:,15)];
  %input = [rawdata.crazyflie_input(:,2:5)+repmat(rawdata.crazyflie_input(:,6),1,4),rawdata.crazyflie_input(:,7)];
  input = [rawdata.crazyflie_extra_input(:,2:5)+repmat(rawdata.crazyflie_extra_input(:,6),1,4),rawdata.crazyflie_extra_input(:,7)];
  t = rawdata.crazyflie_state_estimate(:,15);
  posdata = pos((t>t0)&(t<tf),1:6);
  
  t = t((t>t0)&(t<tf));
  
  input1zoh = PPTrajectory(zoh(input(:,5),input(:,1)'));
  input2zoh = PPTrajectory(zoh(input(:,5),input(:,2)'));
  input3zoh = PPTrajectory(zoh(input(:,5),input(:,3)'));
  input4zoh = PPTrajectory(zoh(input(:,5),input(:,4)'));
  input1 = input1zoh.eval(t);
  input2 = input2zoh.eval(t);
  input3 = input3zoh.eval(t);
  input4 = input4zoh.eval(t);

  inputdata = [input1,input2,input3,input4];
  
  posdata(:,4:6) = unwrap(posdata(:,4:6));
  
  data = [t,inputdata,posdata];
  save(['clean' num2str(i) '.mat'],'data');
end

% you can remove some experiments from the sysid here
% ex: files = [1 3 4]
%files = 1:size(T,1);
%files = [1 2 3 4 5 7 9 10 11];
%files = [6 8];
files = [3];

d = cell(1,numel(files));
for i=1:numel(files)
  
  % Load clean data
  load (['clean' num2str(files(i)) '.mat']);
  t = data(:,1);
  udata = data(:,2:5);
  xdata = data(:,6:11);

  % Fit PPtrajectory 
  xdata = PPTrajectory(spline(t,xdata'));
  udata = PPTrajectory(zoh(t,udata'));

  % Sample at uniform rate
  dt = 1/100;
  t_sample = t(1):dt:t(end); 

  inputs = udata.eval(t_sample);
  
  %xyzoutputs = xdata.eval(t_sample);
  %xyzoutputs = xyzoutputs(1:3,:);
  % you can shift the gyro w.r.t vicon here
  %gyrooutputs = xdata.eval(t_sample);
  %gyrooutputs = gyrooutputs(4:6,:);
  %[b,a] = butter(1,0.2);
  %gyrooutputs = filtfilt(b,a,gyrooutputs')';
  %outputs = [xyzoutputs;gyrooutputs];
  outputs = xdata.eval(t_sample);
  
  sysiddata = iddata(outputs',inputs',dt);
  %set(sysiddata,'InputName',{'thrust1','thrust2','thrust3','thrust4'},'OutputName',{'x','y','z','gyrox','gyroy','gyroz'});
  set(sysiddata,'InputName',{'thrust1','thrust2','thrust3','thrust4'},'OutputName',{'x','y','z','roll','pitch','yaw'});

  d{i} = sysiddata;
end

z = merge(d{:});

% Shift data to take into account delay
% (delay is 28ms)
delay = round(0.028/dt);
z = nkshift(z,delay*ones(1,4));

save('sysidData.mat','z');