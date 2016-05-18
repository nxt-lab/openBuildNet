% This script runs the global/aggregator node.
%
% This file is part of the openBuildNet simulation framework developed at
% EPFL.
%
% Authors: Truong X. Nghiem (xuan.nghiem@epfl.ch)

Nnode = 4;
Nhorizon = 5;
totalSteps = 50;

Ref = (rand(totalSteps, 1) - 0.5) * Nnode / 2;
rho = 10000;

node = Aggregator(Nnode, rho, Nhorizon, Ref);

fprintf('Starting the aggregator: please start all other nodes then the SMN...');
% For debugging purposes, we shall run the simulation with a large timeout
% value (so that if some other node hangs, this one will eventually time
% out and will not hang. We will also turn off stopping the simulation if
% timeout, so that later on we can resume the simulation if desired.
b = node.runSimulation(120, false);
fprintf('Simulation stopped with result (1/true means success): %d.\n', b);

% delete the node variable to close it
% clear node
