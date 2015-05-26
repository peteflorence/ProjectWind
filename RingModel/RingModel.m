% This model is a simple flying ellipsoid, inspired from Sullivan 2008 

% Paramters

R_0 = 0.284/2; % 28.4 cm, measured exit diameter of vortex ring
L = 0.791;     % 79.1 cm, measured length of vortex cannon

% Calculate ellipsoid radii, assuming a ratio of semi-minor to semi-major
% axes of gamma = 0.828
gamma = 0.828;

% Calculate semi-major axis

R = (3 * (R_0)^2 * L / (4 * gamma) ) ^ (1/3)


% Calculate semi-minor axis

Rgamma = R * gamma