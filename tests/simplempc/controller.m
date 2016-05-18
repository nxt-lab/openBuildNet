% The controller node
% This file is part of the openBuildNet framework developed at EPFL.
% (C) 2016 LA-EPFL
% Author: Truong X. Nghiem (truong.nghiem@gmail.com, xuan.nghiem@epfl.ch)

% See: http://users.isy.liu.se/johanl/yalmip/pmwiki.php?n=Examples.StandardMPC

% Model data
A = [2. -1.; 1. 0.2];
B = [1.; 0.];
x0 = [3.; 1.];  % Initial state

% MPC data
nx = size(A,2); % Number of states
nu = size(B,2); % Number of inputs

% MPC data
Q = eye(nx);
R = 2.;
N = 7;

% Prepare the optimization model
tic
u = sdpvar(repmat(nu,1,N),ones(1,N));
x = sdpvar(repmat(nx,1,N+1),ones(1,N+1));

constraints = [];
objective = 0;
for k = 1:N
    objective = objective + norm(Q*x{k},1) + norm(R*u{k},1);
    constraints = [constraints, x{k+1} == A*x{k} + B*u{k}];
    constraints = [constraints, -1 <= u{k}<= 1, -5<=x{k}<=5];
end

mpccontroller = optimizer(constraints, objective, sdpsettings('solver', 'glpk'),x{1},[u{:}]);
toc

% Create the node
node = OBNNode('controller', 'simplempc', 'mqtt');

% Create the ports
node.createInputPort('x', 'v', 'double');   % feedback
node.createOutputPort('u', 'v', 'double'); % control values

% Create Callbacks
node.addCallback(@() disp('INIT'), 'INIT');
node.addCallback(@() disp('TERMINATE'), 'TERM');

MAINBLOCK = 0;

global US
US = [];

node.addCallback(@controller_opt, 'Y', MAINBLOCK, node, mpccontroller);

disp('Ready to start simulation...');
status = node.runSimulation(20, false);
fprintf('Final status: %d\n', status);

% Plot the results
if status
    plot(US(1,:));
end
