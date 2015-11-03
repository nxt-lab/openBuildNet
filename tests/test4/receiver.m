% Receiver node in Matlab.
% This is similar to the C++ node receiver.cpp
% Run this node with runreceiver.m.
%
% This file is part of the openBuildNet simulation framework developed at
% EPFL.
%
% Authors: Truong X. Nghiem (xuan.nghiem@epfl.ch)

classdef receiver < OBNNode
    properties (Constant)
        % Define the indices of the updates
        MAIN_UPDATE = 0;    % the main, regular update
        
    end
    
    methods
        function obj = receiver()
            % Node 'ctr' in workspace 'test2'
            obj = obj@OBNNode('receiver', 'test4', 'mqtt');
            
            % Create input ports
            obj.createInputPort('u1', 's', 'double', 'strict', true);
            obj.createInputPort('u2', 'v', 'double', 'strict', true);
            obj.createInputPort('u3', 's', 'double', 'strict', true);
            
            % Register callbacks
            obj.addCallback(@obj.print_inputs, 'X', receiver.MAIN_UPDATE);

            obj.addCallback(@obj.onInit, 'INIT');
            obj.addCallback(@obj.simple_callback, 'TERM', 'Simulation is terminated');
            
            % Set callbacks for u1 and u3 when they receive values
            % The callbacks are only called on the main / Matlab thread, unline the C++ version
            % which can run a callback on the communication thread (immediately at the event).
            obj.addCallback(@obj.print_input, 'RCV', 'u1', 'u1');  % The second name is for the
                                                                   % callback
            obj.addCallback(@obj.print_input, 'RCV', 'u3', 'u3');
        end
        
    end
    
    methods (Access=private)
        % These are the callbacks
        function simple_callback(this, s)
            fprintf('At %d %s\n', this.currentSimTime, s);
        end
        
        function onInit(this)
            fprintf('At %d INIT\n', this.currentSimTime);
        end
        
        function print_inputs(this)
            fprintf('At %d UPDATE_X\n', this.currentSimTime());
            if this.inputPending('u2')
                disp('u2:');
                while this.inputPending('u2')
                    disp(this.input('u2')');
                end
            end
        end
        
        function print_input(this, portname)
            if this.inputPending(portname)
                fprintf('%s: %g\n', portname, this.input(portname));
            end
        end
        
    end
end
