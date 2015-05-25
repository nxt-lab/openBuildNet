% This script runs the controller node which is implemented in nodectrl.m
% Before running this script, make sure that yarpserver is running.
% Do not run the SMN program before running this script.
% Follow the instructions displayed by this script.
%
% This file is part of the openBuildNet simulation framework developed at
% EPFL.
%
% Authors: Truong X. Nghiem (xuan.nghiem@epfl.ch)

disp('Please check that yarpserver is running. Press any key to continue...');
pause

ctrlnode = nodectrl();
disp('Starting the controller node: please start all other nodes then the SMN...');
b = ctrlnode.runSimulation();
fprintf('Simulation stopped with result (1/true means success): %d.\n', b);

disp('Now plotting the result...');
ctrlnode.plot();
