% Define a class that implements a local node of distributed optimization.
% Run this node with runlocal.m.
%
% This file is part of the openBuildNet simulation framework developed at
% EPFL.
%
% Authors: Truong X. Nghiem (xuan.nghiem@epfl.ch)

classdef localController < OBNNode
    properties (Constant)
        % Define the indices of the updates
        OBTAINSTATE_UPDATE = 0;    % the update to get the state values from plant
        CONTROL_UPDATE = 1;             % the update to send the control values to plant
        
        % the workspace is the same for all nodes
        WORKSPACE = 'disttracking';
        
        COMM_TYPE = 'mqtt';             % the communication type, yarp or mqtt
        COMM_PARAMS = 'tcp://128.178.5.139:1883';               % the communication parameter
        
        % Any constant parameters (for all local nodes) can be placed here
    end
    
    % Internal variables of this particular node, e.g. state variables
    properties (Access=private)
        Nhorizon
        model
        xlim
        ulim
        mpc
        u
        U
        Gamma
        delta
        x0
        
        admm_events = [false, false];   % to keep track of receipts of [Gamma, delta]
    end
    
    methods
        function obj = localController(id,solver, Nhorizon, A,B,xLim,uLim,varargin)
            % id = the unique ID of this node
            % Nhorizon =  length of horizon for reference tracking
            % model = struct with system matrices for the plant
            % constarints = stagewise constraints for the plant, fields xlim,ulim
            % constraints.xlim = [xmin, xmax] = state constraints for plant
            % constraints.ulim = [umin, umax] = input constraints
            % Q = Quadratic parameter for local stage cost
            % c = Affine parameter for local stage cost
            % varargin = cost parameters R,cu,Q,cx,P,cxN
            
            
            % create the node
            if ischar(id)
                nodename = ['Controller' id];
            elseif isnumeric(id)
                nodename = ['Controller' int2str(id)];
            else
                error('Invalid node ID.');
            end
            
            obj = obj@OBNNode(nodename, localController.WORKSPACE, localController.COMM_TYPE, ...
                              localController.COMM_PARAMS);
            
            % Create inputs into this node. The syntax for each input is:
            %   obj.createInputPort('name_of_port', container, type);
            % where 'name_of_port' must be a unique valid name; container
            % is 's' for scalar, 'v' for vector, and 'm' for matrix; type
            % must be a valid Matlab type name, typically 'double' but can
            % also be 'logical', 'int64', 'int32', 'uint64', 'uint32'
            
            % set PROPERTIES:
            obj.Nhorizon = Nhorizon;
            obj.model.A = A;
            obj.model.B = B;
            obj.model.xLim = xLim;
            obj.model.uLim = uLim;
            %nx = size(A,1);
            %nu = size(B,2);
            
            obj.mpc = obj.MPCsolver(Nhorizon,solver,obj.model,varargin{:});

            % INPUTS:
            % Quadratic parameter for optimization cost
            obj.createInputPort('Gamma', 'm', 'double');
            % Affine parameter for optimization cost
            obj.createInputPort('delta', 'v', 'double');
            % current state of the plant
            obj.createInputPort('x', 's', 'double');
            
            % OUTPUTS:
            % Control input to the plant
            obj.createOutputPort('u', 's', 'double');
            % Primary decision variable to be passed to the aggregator
            % during ADMM iterations
            obj.createOutputPort('U', 'v', 'double');
            
            % Register callbacks
            obj.addCallback(@obj.obtainstate, 'Y', localController.OBTAINSTATE_UPDATE);
            obj.addCallback(@obj.computecontrol, 'Y', localController.CONTROL_UPDATE);
            obj.addCallback(@obj.admmiter, 'RCV', 'Gamma', 1); % 1 means Gamma has been received
            obj.addCallback(@obj.admmiter, 'RCV', 'delta', 2); % 2 means delta has been received
                        
            obj.addCallback(@obj.onInit, 'INIT');
            obj.addCallback(@obj.simple_callback, 'TERM', 'Simulation is terminated');
        end
        
    end
    
    methods (Access=private)
        
        % optimizer construction
        function opt  = MPCsolver(this,N,solver,model,varargin)
            % creates MPC solver
            % varargin = cost parameters R,cu,Q,cx,P,cxN
            
            nx = size(this.model.A,1);
            nu = size(this.model.B,2);
            
                        
            % CONSTRAINTS
            
            % varargin = R,cu,Q,cx,P,cxN
            numvarargs = length(varargin);
            if numvarargs > 6
                error('too many arguments');
            end
            % defualt values
            optargin = {zeros(nu),zeros(nu,1),...
                zeros(nx),zeros(nx,1),...
                zeros(nx),zeros(nx,1)};
            % replace default values with provided ones
            optargin(1:numvarargs) = varargin;         
            
            [R,cu,...
             Q,cx,...
             P,cxN] = optargin{:};
            
            x0 = sdpvar(nx,1,'full');
            X = sdpvar(N,nx,'full');
            Umpc = sdpvar(N,nu,'full');
            u0mpc = Umpc(1,:)';
            Gamma = sdpvar(N*nu,N*nu,'full');
            delta = sdpvar(N*nu,1,'full');
            %
            xmin = this.model.xLim(1); xmax = this.model.xLim(2);
            umin = this.model.uLim(1); umax = this.model.uLim(2);
            
            con = [];
            if ~isempty(xmax)&&~isempty(xmin)
                con = con + [xmin <= X , X <= xmax , umin <= Umpc , Umpc <= umax ];
            end
            if ~isempty(umax)&&~isempty(umin)
                con = con + [X' == model.A*[x0 , X(1:end-1,:)'] + model.B*Umpc' ]; % dynamics
            end
            % COST
            
            Hx = blkdiag(kron(eye(N-1),Q),P);
            hx = [kron(ones(N-1,1),cx);cxN];
            Hu = kron(eye(N),R);
            hu = kron(ones(N,1),cu);
            cost = 0.5 * Umpc'*Hu*Umpc + hu'*Umpc + ...
                   0.5 * X'*Hx*X + hx'*X + ...
                   0.5 * Umpc'*Gamma*Umpc + delta'*Umpc ;
            
            %
            ops = sdpsettings('verbose',1,'solver',solver);
            
            opt = optimizer(con,cost,ops,{x0,Gamma,delta},{u0mpc,Umpc});
            
        end
                
        
        % These are the callbacks
        function simple_callback(this, s)
            fprintf('At %d %s\n', this.currentSimTime, s);
        end
        
        function onInit(this)
            % This function is called everytime the simulation starts, so
            % use it to initialize the state and outputs of this node.
            % Although it's not required, all outputs of this node should
            % be initialized properly here.
            
            % Here we set the initial power input of the battery to some
            % value
            this.u = 0;
            this.U = zeros(this.Nhorizon, 1);
            this.output('u', this.u);
            this.output('U', this.U);
            
            fprintf('At %d INIT\n', this.currentSimTime);
        end
        
        function obtainstate(self)
        % Get the state from plant and save it, then send U to aggregator
            
            %disp(['Obtain state at ', num2str(self.currentSimTime)]);
            self.x0 = self.input('x');
            % self.output('U', self.U);
        end
        
        function admmiter(this, whichinput)
        % This is the main computation function of this node.
        % It performs the iteration of the distributed algorithm and is
        % called each and every time the global node sends either Gamma or delta to the local
        % node.
        % whichinput = 1 for Gamma and 2 for delta.
        % Once both have been received, it should compute U and send U to the aggregator.
            switch whichinput
              case 1
                this.Gamma = this.input('Gamma');
              case 2
                this.delta = this.input('delta');
              otherwise
                warning('Wrong whichinput value in admm iteration callback.');
                return;
            end
            if this.admm_events(whichinput)
                % Already received the input --> should not receive it again
                warning('In ADMM iteration callback, input %d has been received twice.', whichinput);
            end
            this.admm_events(whichinput) = true;
            
            if all(this.admm_events)
                this.admm_events = [false, false]; % reset tracking
                
                % Perform the local iteration computation and send the result out
                [solution,diagnostics] = this.mpc{{this.x0,this.Gamma,this.delta}};
                this.u = solution{1};
                this.U = solution{2};
                
                this.sendSync('U',this.U); % sendSync() must be used inside port event callbacks
            end
            % fprintf('At %d UPDATE_Y\n', this.currentSimTime);
        end
        
        function computecontrol(self)
            self.output('u', self.u);  % Send out the control value to the plant
        end
    end
end
