% Define a class that implements the controller node in Matlab.
% This is similar to the C++ node nodectrl.cpp.
% Run this node with runctrl.m.
%
% This file is part of the openBuildNet simulation framework developed at
% EPFL.
%
% Authors: Truong X. Nghiem (xuan.nghiem@epfl.ch)

classdef nodectrl < OBNNode
    properties (Constant)
        % Define the indices of the updates
        MAIN_UPDATE = 0;    % the main, regular update
        
        % Parameters of the controller
        A = [-0.82, 1.0, 0.82;
            1.0 , 0.0, 0.0;
            0.0 , 1.0, 0.0];
        B = [32, -32;
            0, 0;
            0, 0];
        C = [12.62, -19.75, 7.625];
    end
    
    % Internal variables of the controller
    properties (Access=private)
        x   % Internal state
        logs    % Store [time, setpoint, velocity] in columns
    end
    
    methods
        function obj = nodectrl()
            % Node 'ctr' in workspace 'test2'
            obj = obj@OBNNode('ctrl', 'test2');
            
            % Create input for velocity 'v': a double scalar
            obj.createInputPort('v', 's', 'double');
            
            % Create input for setpoint 'sp': a double scalar
            obj.createInputPort('sp', 's', 'double');
            
            % Create output port for command 'u': a double scalar
            obj.createOutputPort('u', 's', 'double');

            % Register callbacks
            obj.addCallback(@obj.update_state, 'X', nodectrl.MAIN_UPDATE);
            obj.addCallback(@obj.compute_command, 'Y', nodectrl.MAIN_UPDATE);

            obj.addCallback(@obj.onInit, 'INIT');
            obj.addCallback(@obj.simple_callback, 'TERM', 'Simulation is terminated');
        end
        
        function plot(this)
            % Plot results in the log
            if isempty(this.logs)
                return;
            end
            
            figure;
            hold on;
            stairs(this.logs(1,:), this.logs(2,:), 'b');
            stairs(this.logs(1,:), this.logs(3,:), 'r');
            legend('Setpoint', 'Velocity');
            ylabel('Angular Velocity');
            xlabel('Time');
        end
    end
    
    methods (Access=private)
        % These are the callbacks
        function simple_callback(this, s)
            fprintf('At %d %s\n', this.currentSimTime, s);
        end
        
        function onInit(this)
            this.x = zeros(3,1);    % Initial state
            this.output('u', 0);    % Initial output
            this.logs = zeros(3,0); % Clear the log
            fprintf('At %d INIT\n', this.currentSimTime);
        end
        
        function update_state(this)
            % Read inputs to local variables because we will reuse them
            sp = this.input('sp');
            velo = this.input('v');
            
            % Update controller's state
            this.x = this.A * this.x + this.B * [sp; velo];
            
            % Save to log
            t = this.currentSimTime;
            this.logs(:, end+1) = [t; sp; velo];
            
            fprintf('At %d UPDATE_X\n', t);
        end
        
        function compute_command(this)
            % Compute new output
            this.output('u', this.C * this.x);
            
            fprintf('At %d UPDATE_Y\n', this.currentSimTime);
        end
        
    end
end
