% Define a class that implements the global node of distributed optimization.
% Run this node with runglobal.m.
%
% This file is part of the openBuildNet simulation framework developed at
% EPFL.
%
% Authors: Truong X. Nghiem (xuan.nghiem@epfl.ch)

classdef Aggregator < OBNNode
    properties (Constant)
        % Define the indices of the updates
        MAIN_UPDATE = 0;    % the main update that performs the ADMM iterations
        
        % the workspace is the same for all nodes
        WORKSPACE = 'disttracking';
        
        COMM_TYPE = 'mqtt';             % the communication type, yarp or mqtt
        COMM_PARAMS = '';               % the communication parameter

        % Any constant parameters can be placed here
    end
    
    % Internal variables of this particular node, e.g. state variables
    properties (Access=private)
        Nnode       % The number of local nodes
        Nhorizon    % length of horizon
        Gammas      % quadratic cost parameter to be sent to locals
        deltas      % affine cost parameter to be sent to locals
        Z           % local references stacked in a matrix
        Z_prev      % Z at previous step, used for residual calculation
        MU          % local duals stacked in a matrix
        lambda      % global dual
        rho         % admm stepsize
        T           % summation matrix for references r = Tz
        err         % tracking error
        refHorizon  % reference for the horizon
        reference   % reference trajectory for simulation
        step        % step
        delta_t     % sampling time MIGHT NOT BE NECESSARY
        U           % proposed inputs from locals stacked in a matrix
        epsPrimal = 1e-5; % primal convergence criterion
        epsDual = 1e-5 ;  % dual convergence criterion
        resDual    % residual for dual convergence 
        resPrimal   % residual for primal convergence
        maxIter = 50;
        iter = 0;
        
        admm_events                     % To track receipts from U ports
        in_admm_iters = false;          % true if and only if ADMM iterations are ongoing
    end
    
    methods
        function obj = Aggregator(Nnode, rho, Nhorizon, Ref)
            % N = the number of local nodes, numberred from 1 to N
            assert(isnumeric(Nnode) && isscalar(Nnode) && Nnode >= 1);
            assert(isnumeric(Nhorizon) && isscalar(Nhorizon) && Nhorizon >= 1);
            assert(isnumeric(Ref) && length(Ref) >= Nhorizon)
            
            obj = obj@OBNNode('Aggregator', Aggregator.WORKSPACE, Aggregator.COMM_TYPE, ...
                              Aggregator.COMM_PARAMS);
            obj.Nnode = Nnode;
            obj.Nhorizon = Nhorizon;
            obj.reference = Ref;
            obj.rho = rho;
            obj.lambda = zeros(Nhorizon,1);
            obj.err = zeros(Nhorizon,1);
            obj.T =  kron(ones(1,Nnode),eye(Nhorizon));
            
            obj.Gammas = cell(1,Nnode);
            obj.deltas = cell(1,Nnode);
            obj.U = zeros(Nhorizon,Nnode);
            obj.Z = zeros(Nhorizon,Nnode);
            obj.Z_prev = zeros(Nhorizon,Nnode);
            obj.MU = zeros(Nhorizon,Nnode);

            obj.step = 1;
            
            obj.admm_events = false(Nnode,1);
            
            % Create individual input(s) for each local node to receive
            % values from that node, for example x1 to xN as vectors.
            for k = 1:Nnode
                UportName = ['U' int2str(k)];
                obj.createInputPort(UportName, 'v', 'double');
                obj.createOutputPort(['Gamma'  int2str(k)], 'm', 'double');
                obj.createOutputPort(['delta' int2str(k)], 'v', 'double');
                
                % Add RCV callback on the U port
                obj.addCallback(@obj.admmiter, 'RCV', UportName, k);
            end
            
            % Register callbacks
            obj.addCallback(@obj.main_update, 'Y', obj.MAIN_UPDATE);

            obj.addCallback(@obj.onInit, 'INIT');
            obj.addCallback(@obj.simple_callback, 'TERM', 'Simulation is terminated');
        end
        
    end
    
    methods (Access=private)
        % These are the callbacks
        function simple_callback(this, s)
            fprintf('At %d %s\n', this.currentSimTime, s);
        end
        
        function onInit(this)
            % This function is called everytime the simulation starts, so
            % use it to initialize the state and outputs of this node.
            % Outputs to local nodes can be set here, however not really
            % necessary because they will be calculated by iter_update() at
            % t = 0.
            
            fprintf('At %d INIT\n', this.currentSimTime);
        end
        
        function iter_computation(this)
            % Perform the iteration computation here
            % local dual update
            this.MU = this.MU + this.U - this.Z;
            % local reference update
            G = (eye(this.Nhorizon*this.Nnode) + this.T'*this.T);
            f = this.U(:)+this.MU(:) + this.T'*(this.refHorizon - this.lambda);
            z = G\f;
            this.Z_prev = this.Z;
            this.Z = vec2mat(z,this.Nhorizon)';
            this.err = this.T*z-this.refHorizon ;
            % global dual update
            this.lambda = this.lambda + this.err;
            
            % Write to the output port(s) to send values to the local
            % nodes immediately.
            for k = 1:this.Nnode
                this.Gammas{k} = this.rho*eye(this.Nhorizon);
                this.deltas{k} = this.rho*(-this.Z(:,k) + this.MU(:,k));
                this.sendSync(['Gamma' int2str(k)],this.Gammas{k});
                this.sendSync(['delta' int2str(k)],this.deltas{k});
            end
        end

        
        function admmiter(this, k)
        % Callback of the port events, where k is the index of the U<k> port.
        % It is called every time a local node sends U<i> values to the aggregator.
        % This callback should perform the ADMM iteration whenever all U<i>'s have been
        % received.
        % It should also check the termination condition.
        % The computation should only be performed if and only if this.in_admm_iters =
        % true; and when it finishes, it must reset this variable to stop the ADMM iterations.
            if ~this.in_admm_iters
                return;
            end
            
            %fprintf('admmiter for k = %d\n', k);
            
            % Mark the port event
            if this.admm_events(k)
                warning(['During ADMM iteration, values from port U%d have been received ' ...
                         'multiple times.'], k);
            end
            this.admm_events(k) = true;
            this.U(:,k) = this.input(['U' int2str(k)]); % save the values
            
            % If all have been received, perform the iteration computation
            if all(this.admm_events)
                this.iter = this.iter + 1;
                %fprintf('ADMM iteration %d after receiving from all local nodes.\n', this.iter);
                
                this.admm_events = false(this.Nnode, 1); % reset tracking                    
                
                % the variable continue_iter determines whether to continue
                % convergence criteria
                
                % compute Residuals
                residualRef = this.T*this.Z(:)-this.refHorizon;
                residualLocalTracking = this.U(:)- this.Z(:);
                this.resPrimal = [residualRef;residualLocalTracking];
                %
                this.resDual = -this.rho * reshape( this.Z - this.Z_prev, [], 1 ) ;
                
                % check convergence
                convPrimal = sqrt(this.resPrimal'*this.resPrimal);
                convDual = sqrt(this.resDual'*this.resDual);
                % disp(convPrimal)
                % disp(convDual)
                continue_iter = ~((convPrimal <= this.epsPrimal && convDual <= this.epsDual ) || ...
                                  this.iter >= this.maxIter);

                if continue_iter
                    this.iter_computation();
                else
                    this.iter = 0;

                    % Stop iterations
                    this.in_admm_iters = false;
                    
                    % reset ADMM variables
                    this.U = zeros(this.Nhorizon,this.Nnode);
                    this.Z = zeros(this.Nhorizon,this.Nnode);
                    this.Z_prev = zeros(this.Nhorizon,this.Nnode);
                    this.MU = zeros(this.Nhorizon,this.Nnode);
                end
            end
        end
        
        function main_update(this)
        % This function is called at every major time step.
        % It should start the ADMM iterations; process all port events (to perform the ADMM
        % iterations) until the ADMM algorithm finishes.
            
            % the current time, 0 if this is the initial step
            fprintf('Start ADMM at time %g (s).\n', this.currentSimTime);
            
            this.refHorizon = this.reference(this.step:this.step-1+this.Nhorizon) ; % ASK TRUONG
            this.step = this.step + 1;
            
            % Start ADMM
            this.in_admm_iters = true;

            % Tell local nodes to start iterations
            this.iter_computation();
            
            % Loop to process all port events until it finishes, or a long timeout of 60s.
            mytimeout = tic;
            while (this.in_admm_iters && toc(mytimeout) <= 60)
                if this.processPortEvent(1)
                    mytimeout = tic;
                end
                % We can check this.isRunning()) once in a while to make sure that we don't wait
                % for a long time when the simulation stops unexpectedly
            end
            
            if this.in_admm_iters
                % Must be some error, like a timeout
                this.in_admm_iters = false;
                this.stopSimulation();
                disp('ERROR: An error happened during ADMM iterations; likely a timeout.');
            end
        end
                
    end
end
