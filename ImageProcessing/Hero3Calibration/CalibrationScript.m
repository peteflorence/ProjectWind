load calibration.mat
polyfit(X(:,1),X(:,2),2);

% First column of X is meters
% Second column of X is pixel distance
% Third column of X is predicted with 1-order fit
% Fourth column of X is residual with 1-order fit

% Fifth and six are predicted and residual with 2-order fit


% What we want is a polynomial mapping from pixels to meters
% x = pixels
% y = meters


% First order

p = polyfit(X(:,2),X(:,1),1);

p1 = p(1);
p0 = p(2);

for i = 1:length(X(:,1))

  X(i,3) = p1*X(i,2) + p0;
  X(i,4) = X(i,3) - X(i,1);
  
end

% Plot residuals

figure
plot(X(:,1),X(:,4),'-ko','MarkerSize', 12);
title('1st order residuals')
xlabel('actual (meters)') % x-axis label
ylabel('residual error (meters)') % y-axis label





% Second order

p = polyfit(X(:,2),X(:,1),2);

p2 = p(1);
p1 = p(2);
p0 = p(3);

for i = 1:length(X(:,1))

  X(i,5) = p2*X(i,2)^2 + p1*X(i,2) + p0;
  X(i,6) = X(i,5) - X(i,1);
  
end

% Plot residuals

figure
plot(X(:,1),X(:,6),'-ko','MarkerSize', 12);
title('2nd order residuals')
xlabel('actual (meters)') % x-axis label
ylabel('residual error (meters)') % y-axis label




save calibration.mat
