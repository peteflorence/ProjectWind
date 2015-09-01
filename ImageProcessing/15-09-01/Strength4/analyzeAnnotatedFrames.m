load 001/data.mat


FIRST_LIGHT_ON_FRAME = 33;
if ~(isempty(FIRST_LIGHT_ON_FRAME))
  if ~exist('ALREADY_SHIFTED')
  data(:,1) = data(:,1) - FIRST_LIGHT_ON_FRAME;
  ALREADY_SHIFTED = 1;
  end
end

 
figure
hold on
plot(data(:,1),data(:,2),'.')

% Column 1 is frames, starting from 0
% Column 2 is distance traveled in pixels (x)

% Second order polynomial coefficients, from
% Hero3Calibration/CalibrationScript
p2 = -3.09990751956170e-07;      % Second order coefficient
p1 = -0.00545387283240715;  % First
p0 = 5.99977871157130;      % Constant

data(:,3) = zeros(length(data(:,2)),1);

for i = 1:length(data(:,2))
  
  x = data(i,2);
  data(i,3) = p2*x^2 + p1*x + p0;
  
end




% Plot straight fit from "undistorted image"

figure
plot(data(:,1),data(:,2),'-ko','MarkerSize', 12);
title('"Pixel distance Undistorted image"')
xlabel('frame number') % x-axis label
ylabel('pixel distance') % y-axis label


% Plot mapping to meters using second order fit

figure
plot(data(:,1),data(:,3),'-ko','MarkerSize', 12);
title('Estimated meters traveled using second order fit')
xlabel('frame number') % x-axis label
ylabel('meters distance') % y-axis label


% Add time column
data(:,4) = zeros(length(data(:,2)),1);
for i = 1:length(data(:,2))
  
  x = data(i,1);
  data(i,4) = x / 119.88; %119.88 frames per second
  
end


% Estimate velocity
data(:,5) = zeros(length(data(:,2)),1);
for i = 2:length(data(:,2))
  
  meters = data(i,3) - data(i-1,3);
  time = data(i,4) - data(i-1,4);
  data(i,5) = meters / time; %119.88 frames per second
  
end

% Plot velocity as function of time

figure
plot(data(2:end,4),data(2:end,5),'-ko','MarkerSize', 12);
title('Velocity estimate (finite difference)')
xlabel('time (seconds)') % x-axis label
ylabel('velocity (meters / second)') % y-axis label



save 001/data.mat



