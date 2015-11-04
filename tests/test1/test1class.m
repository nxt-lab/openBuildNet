% A simple test of the node.matlab framework:
% A node is created which simply prints out the messages received from the
% SMN/GC.
% This test creates a node by deriving from OBNNode (rather than using
% simple callbacks as in test1script.m).
% In this test, the node doesn't have any ports, it only communicates with
% the SMN/GC
%
% To run this example:
% - Create an object of this class: mynode = test1class('node1', 'test1');
%   For more options, see OBNNode.
% - Run it: mynode.runSimulation(20) (where 20 is the timeout in seconds)
%
% This file is part of the openBuildNet simulation framework developed at
% EPFL.
%
% Authors: Truong X. Nghiem (xuan.nghiem@epfl.ch)

classdef test1class < OBNNode
    % Define the indices of the updates
    properties (Constant)
        MAIN_UPDATE = 0;    % the main, regular update
        IRREG_UPDATE = 1;   % the irregular update
    end
    
    methods
        function obj = test1class(varargin)
            obj = obj@OBNNode(varargin{:});
            
            % Register callbacks
            obj.addCallback(@obj.simple_callback, 'X', test1class.MAIN_UPDATE, 'UPDATE_X for MAIN_UPDATE');
            obj.addCallback(@obj.simple_callback, 'X', test1class.IRREG_UPDATE, 'UPDATE_X for IRREG_UPDATE');

            obj.addCallback(@obj.ycallback, 'Y', test1class.MAIN_UPDATE);
            obj.addCallback(@obj.simple_callback, 'Y', test1class.IRREG_UPDATE, 'UPDATE_Y for IRREG_UPDATE');

            obj.addCallback(@obj.simple_callback, 'INIT', 'Begin simulation');
            obj.addCallback(@obj.simple_callback, 'TERM', 'Simulation is terminated');
        end
    end
    
    methods (Access=private)
        % These are the callbacks
        function simple_callback(this, s)
            fprintf('At %d %s\n', this.currentSimTime, s);
        end
        
        function ycallback(this)
            t = this.currentSimTime;
            fprintf('At %d UPDATE_Y for MAIN_UPDATE\n', t);

            % randomly request future update
            if rand() >= 0.5
                future = t + randi([11 16], 1);
                r = this.requestFutureUpdate(future, test1class.IRREG_UPDATE, 10);
                if r == 0
                    fprintf('At %d Event request was accepted for time %d.\n', t, future);
                else
                    fprintf('At %d Event request failed with error = %d.\n', t, r);
                end
            end
        end
        
    end
end
