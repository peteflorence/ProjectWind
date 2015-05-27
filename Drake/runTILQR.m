function runTILQR




r_temp = Quadrotor();
%r_temp = addOcean(r_temp, [.8,.45,1.25], [.20;2.5], pi/4);
v = constructVisualizer(r_temp);
r = QuadWindPlant(); % Quadrotor constructor
r.windfield = 'flyingellipsoid';
r.ellipsoidcenter = [3 0 1];

% Create TILQR controller for QuadWindPlant with no wind

x0 = Point(getStateFrame(r_temp));  % initial conditions: all-zeros
x0.base_z = 1; % lift the quad off the ground
u0 = double(nominalThrust(r_temp));
Q = 10 * [eye(3) zeros(3,9) ; zeros(9,3) zeros(9)];
R = eye(4);

%c = tilqr(r_temp,x0,u0,Q,R);


% the linearized system 
[A,B] = linearize(r_temp,0,double(x0),double(u0));

Q = diag([10*ones(6,1); ones(6,1)]);
R = .1*eye(4);
K = lqr(full(A),full(B),Q,R);
K = [K zeros(4,1)];

x0 = Point(getStateFrame(r));  % initial conditions: all-zeros
x0.z = 1; % lift the quad off the ground
u0 = double(nominalThrust(r));

% u = u0 - K*(x-x0)
c = AffineSystem([],[],[],[],[],[],[],-K,double(u0) + K*double(x0));
c = setInputFrame(c,getStateFrame(r));
c = setOutputFrame(c,getInputFrame(r));

sys = feedback(r,c);





% 
% r2 = QuadWindPlant_wVortexRing();
% r2 = r2.setOutputFrame(r.getOutputFrame);
% r2 = r2.setInputFrame(r.getInputFrame);
% 
% % CREATE FEEDBACK CONTROLLER
% 
% % For no Gaussian noise (simulating on same plant):
% %sys = feedback(r,ltvsys);
% 
% % For Gaussian noise (simulating on different plant):
% sys = feedback(r2,c);

disp('done!');

% Cascade if desired
%tic;
%sys = cascade(utraj, r);
%toc;

% Simulate the result
tf = 3;
tic;
disp('Simulating the system...');
xtraj_sim = simulate(sys,[0 tf],x0);
toc;
disp('done!');

% Draw the TVLQR result
xtraj_sim = xtraj_sim(1:12);
xtraj_sim = xtraj_sim.setOutputFrame(r_temp.getStateFrame());
v.playback(xtraj_sim, struct('slider', true));

v.inspector;

end
