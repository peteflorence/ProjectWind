load data.mat

figure
hold on
plot(data(:,7),data(:,8),'.')

% Column 7 is frames, starting from 0
% Column 8 is distance traveled in pixels (x)

% Second order polynomial coefficients, from
% Hero4Calibration/CalibrationScript
p2 = -2.413440479470594e-07;      % Second order coefficient
p1 = 0.004166909976728;  % First
p0 = 0.001899665810442;      % Constant

data(:,9) = zeros(length(data(:,8)),1);

for i = 1:length(data(:,8))
  
  x = data(i,8);
  data(i,9) = p2*x^2 + p1*x + p0;
  
end




% Plot straight fit from "undistorted image"

figure
plot(data(:,7),data(:,8),'-ko','MarkerSize', 12);
title('"Pixel distance Undistorted image"')
xlabel('frame number') % x-axis label
ylabel('pixel distance') % y-axis label


% Plot mapping to meters using second order fit

figure
plot(data(:,7),data(:,9),'-ko','MarkerSize', 12);
title('Estimated meters traveled using second order fit')
xlabel('frame number') % x-axis label
ylabel('meters distance') % y-axis label


% Add time column
data(:,10) = zeros(length(data(:,8)),1);
for i = 1:length(data(:,8))
  
  x = data(i,7);
  data(i,10) = x / 119.88 %119.88 frames per second
  
end


% Estimate velocity
data(:,11) = zeros(length(data(:,8)),1);
for i = 2:length(data(:,8))
  
  meters = data(i,9) - data(i-1,9);
  time = data(i,10) - data(i-1,10);
  data(i,11) = meters / time %119.88 frames per second
  
end

% Plot velocity as function of time

figure
plot(data(:,10),data(:,11),'-ko','MarkerSize', 12);
title('Velocity estimate (finite difference)')
xlabel('time (seconds)') % x-axis label
ylabel('velocity (meters / second)') % y-axis label



save data.mat



