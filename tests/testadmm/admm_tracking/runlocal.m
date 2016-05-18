% This script runs the local nodes. Each local node requires a unique ID.
% The name of the local node will be /disttracking/Controller<ID>
%
% This file is part of the openBuildNet simulation framework developed at
% EPFL.
%
% Authors: Truong X. Nghiem (xuan.nghiem@epfl.ch)

% We will run all local nodes in one Matlab instance.

Nlocals = 4;

Nhorizon = 5;
A = 1;
B = 1;
xLim = [0, 20];
uLim = [-1, 1];

nodes = OBNNode.empty(1,0);
for ID = 1:Nlocals
    nodes(ID) = localController(ID, 'gurobi', Nhorizon, A, B, xLim,uLim,0.1,1);
end

fprintf('Starting the local nodes: please start all other nodes then the SMN...\n', ID);

% For debugging purposes, we shall run the simulation with a large timeout
% value (so that if some other node hangs, this one will eventually time
% out and will not hang. We will also turn off stopping the simulation if
% timeout, so that later on we can resume the simulation if desired.
b = runSimulation(nodes, 30, false);
disp('Simulation stopped with result (1/true means success):');
disp(b);

% delete the node variable to close it
% clear nodes
