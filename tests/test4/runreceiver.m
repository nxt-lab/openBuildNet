% Follow the instructions displayed by this script.
%
% This file is part of the openBuildNet simulation framework developed at
% EPFL.
%
% Authors: Truong X. Nghiem (xuan.nghiem@epfl.ch)

disp('Please check that MQTT server is running. Press any key to continue...');
pause

receivernode = receiver();
disp('Starting the receiver node: please start all other nodes then the SMN...');
b = receivernode.runSimulation(30);
fprintf('Simulation stopped with result (1/true means success): %d.\n', b);
