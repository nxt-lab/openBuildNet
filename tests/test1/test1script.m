% A simple test of the node.matlab framework:
% A node is created which simply prints out the messages received from the
% SMN/GC.
% This test uses callbacks and does not derive a new node class from
% OBNNode, which is suitable for a quick implementation in a Matlab script
% file. Another approach, which is more appropriate for a complex node with
% internal states and resources, is to implement the node in a child class
% of OBNNode. See test1class.m for details.
%
% This file is part of the openBuildNet simulation framework developed at
% EPFL.
%
% Authors: Truong X. Nghiem (xuan.nghiem@epfl.ch)

% Create a node named 'node1' in the workspace 'test1', so the full path of
% this node will be /test1/node1
mynode = OBNNode('node1', 'test1');

% In this test, the node doesn't have any ports, it only communicates with
% the SMN/GC

% There will be two updates: MAIN_UPDATE with id 0 and IRREG_UPDATE with id
% 1.  The IDs of updates must be unique, and they are from 0 to
% OBNNode.maxUpdateID (typically 63).
MAIN_UPDATE = 0;
IRREG_UPDATE = 1;

% Add callback for UPDATE_X events
mynode.addCallback(@() fprintf('At %d UPDATE_X for MAIN_UPDATE\n', mynode.currentSimTime), 'X', MAIN_UPDATE);
mynode.addCallback(@() fprintf('At %d UPDATE_X for IRREG_UPDATE\n', mynode.currentSimTime), 'X', IRREG_UPDATE);

% Add callback for UPDATE_Y events
fsimple = @(s) fprintf('At %d %s\n', mynode.currentSimTime, s);
% Here we use a custom argument as the name of the update, so that both
% callbacks can be bound to the same function, with different custom
% arguments.
mynode.addCallback(@() test1yfunc(mynode,IRREG_UPDATE), 'Y', MAIN_UPDATE);
mynode.addCallback(fsimple, 'Y', IRREG_UPDATE, 'UPDATE_Y for IRREG_UPDATE');

% Add callbacks for INIT and TERMINATE
mynode.addCallback(fsimple, 'INIT', 'Begin simulation');
mynode.addCallback(fsimple, 'TERM', 'Simulation is terminated');

disp('Starting simulation; please start the SMN server!');
b = mynode.runSimulation(20);
disp('Simulation stopped with result:');
disp(b);
