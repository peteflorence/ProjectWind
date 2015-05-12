load data.csv

figure
hold on
plot(data(:,7),data(:,8),'.')

% Column 7 is frames, starting from 0
% Column 8 is distance traveled in pixels (x)

% Second order polynomial coefficients, from
% Hero4Calibration/CalibrationScript
p2 = 4.440476190476176;      % Second order coefficient
p1 = 2.373214285714287e+02;  % First
p0 = 1.309523809523798;      % Constant

data(:,9) = zeros(length(data(:,8)),1);

for i = 1:length(data(:,8))
  
  x = data(i,8);
  data(i,9) = p2*x^2 + p1*x + p0;
  
end

