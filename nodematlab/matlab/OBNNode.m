classdef OBNNode < handle
    % Class that implements an openBuildNet node in Matlab.
    %
    % This file is part of the openBuildNet simulation framework developed
    % at EPFL.
    %
    % Author: Truong X. Nghiem (xuan.nghiem@epfl.ch)
    
    % Last update: 2015-05-19
    
    properties (Constant)
        MaxUpdateID = obnsim_yarp_('maxUpdateID');
    end
    
    methods (Access=protected)
        % These are the event handler for events sent from the MEX (node
        % object).
        function onUpdateY(this, mask)
            k = 1;
            while mask ~= 0 && k <= length(this.callbacks_update_y)
                if bitget(mask, this.callbacks_update_y(k).id+1) == 1
                    this.callbacks_update_y(k).func(this.callbacks_update_y(k).args{:});
                    mask = bitset(mask, this.callbacks_update_y(k).id+1, 0);    % reset that bit
                end
                k = k + 1;
            end
        end
        
        function onUpdateX(this, mask)
            k = 1;
            while mask ~= 0 && k <= length(this.callbacks_update_x)
                if bitget(mask, this.callbacks_update_x(k).id+1) == 1
                    this.callbacks_update_x(k).func(this.callbacks_update_x(k).args{:});
                    mask = bitset(mask, this.callbacks_update_x(k).id+1, 0);    % reset that bit
                end
                k = k + 1;
            end
        end
        
        function onSimInit(this)
            if ~isempty(this.callback_init)
                this.callback_init.func(this.callback_init.args{:});
            end
        end
        
        function onSimTerm(this)
            if ~isempty(this.callback_term)
                this.callback_term.func(this.callback_term.args{:});
            end
        end
    end
    
    methods
        function this = OBNNode(nodeName, ws)
            %       obj = OBNNode(nodeName, ws)
            % Create a new openBuildNet node in Matlab with a given name
            % and optionally a workspace name. The full name of the node in
            % the network will be /ws/nodeName
            
            narginchk(1, 2);
            [b, nodeName] = OBNNode.isValidIdentifier(nodeName);
            assert(b, 'Invalid node''s name.');
            if nargin > 1 && ~isempty(ws)
                [b, ws] = OBNNode.isValidIdentifier(ws);
                assert(b, 'Invalid workspace name.');
            else
                ws = '';
            end
            
            % Create a new node object and save its pointer in id_
            this.id_  = obnsim_yarp_('nodeNew', nodeName, ws);
            
            % Create the maps from nodes' names to their IDs
            this.inputPorts = containers.Map('KeyType', 'char', 'ValueType', 'int32');
            this.outputPorts = containers.Map('KeyType', 'char', 'ValueType', 'int32');
        end
        
        function delete(this)
            obnsim_yarp_('nodeDelete', this.id_);
        end
        
        function createInputPort(this, portName, containerType, elementType, varargin)
            %   createInputPort(this, portName,
            %                   containerType, elementType, ...)
            %
            % Create a new physical input port with a given name and type.
            % - portName is the port's name inside the node
            % - containerType is a character specifying whether the port
            %   receives a scalar ('s'), a vector ('v'), a matrix ('m'), or
            %   any binary data ('b').
            % - elementType is a string specifying the date type of the
            %   elements in the container, except for binary type. Valid
            %   values are: 'logical', 'double', 'int32', 'uint32',
            %   'int64', 'uint64'.
            %
            % Optional arguments (name-value pair)
            %   'strict'  Is the port strict in reading? (default: false)
            %
            % Note that port is automatically opened, so the network (in
            % particular the Yarp server) must be present and working
            % properly.
            
            narginchk(3, inf);
            assert(isscalar(this));
            
            if nargin < 4 || isempty(elementType)
                elementType = '';
            end;
            [portName, elementType] = this.checkPortSettings(portName, containerType, elementType);
            
            id = obnsim_yarp_('createInput', this.id_, containerType, elementType, portName, varargin{:});
            assert(id >= 0, 'Could not create and open the input port.');

            % save the port to the map
            this.inputPorts(portName) = id;
        end
        
        function createOutputPort(this, portName, containerType, elementType)
            %   createOutputPort(this, portName,
            %                   containerType, elementType)
            %
            % Create a new physical output port with a given name and type.
            % - portName is the port's name inside the node
            % - containerType is a character specifying whether the port
            %   receives a scalar ('s'), a vector ('v'), a matrix ('m'), or
            %   any binary data ('b').
            % - elementType is a string specifying the date type of the
            %   elements in the container, except for binary type. Valid
            %   values are: 'logical', 'double', 'int32', 'uint32',
            %   'int64', 'uint64'.
            %
            % Note that port is automatically opened, so the network (in
            % particular the Yarp server) must be present and working
            % properly.
            
            narginchk(3, inf);
            assert(isscalar(this));
            
            if nargin < 4 || isempty(elementType)
                elementType = '';
            end;
            [portName, elementType] = this.checkPortSettings(portName, containerType, elementType);
            
            id = obnsim_yarp_('createOutput', this.id_, containerType, elementType, portName);
            assert(id >= 0, 'Could not create and open the output port.');

            % save the port to the map
            this.outputPorts(portName) = id;
        end
        
        function d = input(this, portName)
           %    d = input(this, portName)
           % reads from a non-strict physical input port.
           % portName is the name of the input port
           % Returns the current value of the port, in an appropriate
           % Matlab type.
           % This function will cause an error if the port doesn't exist,
           % if it's not an appropriate port type.
           % 
           % Reading value from an input port can be expensive as it
           % involves calling a MEX function and transferring data from the
           % MEX to Matlab. Hence it's a good practice to read from an
           % input port once to a local variable to use, rather than
           % reading from the port repeatedly.
           
           narginchk(2, 2);
           assert(isscalar(this));
           assert(this.isValidIdentifier(portName), 'Port name is invalid.');
           assert(this.inputPorts.isKey(portName), 'Input port does not exist.');
           
           d = obnsim_yarp_('readInput', this.id_, this.inputPorts(portName));
        end
        
        
        function output(this, portName, d)
           %    output(this, portName, d)
           % sets the value of a physical output port.
           % - portName is the name of the output port
           % - d is the value of an appropriate type (e.g. numeric for
           %    a numeric port).
           % The value will NOT be sent immediately but is only stored in
           % the port, to be sent at the end of the current callback
           % (typically UPDATE_Y but also INIT...).
           %
           % This function will cause an error if the port doesn't exist,
           % if it's not an appropriate port type, or if the value d is not
           % of an appropriate type that can be converted to the type of
           % the port.
           % 
           % Writing a value to an output port can be expensive as it
           % involves calling a MEX function and transferring data from
           % Matlab to the MEX. Hence it's a good practice to use a local
           % variable for the output, and only write it to the port once at
           % the end.
           
           narginchk(3, 3);
           assert(isscalar(this));
           assert(this.isValidIdentifier(portName), 'Port name is invalid.');
           assert(this.outputPorts.isKey(portName), 'Output port does not exist.');
           
           obnsim_yarp_('writeOutput', this.id_, this.outputPorts(portName), d);
        end
        
        function sendSync(this, portName, d)
           %    sendSync(this, portName, d)
           % requests a physical output port to send its value
           % synchronously (i.e. the function blocks until the message is
           % sent). If d (optional) is given, the port's value will be set
           % to d before it's sent; otherwise the current value is sent.
           % - portName is the name of the output port
           % - Optionally, d is the value of an appropriate type (e.g.
           %    numeric for a numeric port).
           %
           % Note that typically, ports with updated values will
           % automatically send their new values at the end of an event
           % callback. This applies to all events that, in the openBuildNet
           % document, are supposed to compute and send out outputs, in
           % particular UPDATE_Y and INIT. In these cases, there is no need
           % to call sendSync() explicitly.
           %
           % This function will cause an error if the port doesn't exist,
           % if it's not an appropriate port type, or if the value d is not
           % of an appropriate type that can be converted to the type of
           % the port.

           narginchk(2, 3);
           assert(isscalar(this));
           assert(this.isValidIdentifier(portName), 'Port name is invalid.');
           assert(this.outputPorts.isKey(portName), 'Output port does not exist.');
           
           if nargin > 2
               % Set the value first
               obnsim_yarp_('writeOutput', this.id_, this.outputPorts(portName), d);
           end
           obnsim_yarp_('sendSync', this.id_, this.outputPorts(portName));
        end
        
        function s = listInputs(this)
            % List all input ports either to the console or to an output
            % variable.
            narginchk(1, 1);
            assert(isscalar(this));

            ss = this.inputPorts.keys();
            if nargout == 0
                for k = 1:length(ss)
                    disp(ss{k});
                end
            else
                s = ss;
            end
        end
        
        function s = listOutputs(this)
            % List all output ports either to the console or to an output
            % variable.
            narginchk(1, 1);
            assert(isscalar(this));
            
            ss = this.outputPorts.keys();
            if nargout == 0
                for k = 1:length(ss)
                    disp(ss{k});
                end
            else
                s = ss;
            end
        end
        
        function info = getPortInfo(this, portName)
            %   info = getPortInfo(this, portName)
            % Returns information about the given port(s).
            % - portName is either a port's name (a string) or a cell array
            %   of ports' names (for multiple ports).
            % - info is a structure array, one record for each port,
            %   containing information about the ports. It has the
            %   following fields:
            %   + type: a character of 'i' for input, 'o' for output, and
            %           'd' for data port
            %   + container: a character specifying the container type of
            %       the value of the port: 's' for scalar, 'v' for vector,
            %       'm' for matrix, and 'b' for binary data (string).
            %   + element: a string specifying the element type (except for
            %       binary data (container = 'b'); see the createInputPort
            %       or createOutputPort functions for details.
            %   + strict: a logical value specifying whether the input/data
            %       port uses strict reading. Does not apply for output
            %       ports.
            narginchk(2, 2);
            assert(isscalar(this));
            
            if ischar(portName)
                portName = {portName};
            else
                assert(iscell(portName), 'portName must be either a string or a cell array of strings.');
            end
            assert(~isempty(portName), 'portName is empty.');
            portIDs = zeros(length(portName), 1);
            for k = 1:length(portName)
                assert(this.isValidIdentifier(portName{k}), 'Port name #%d is invalid.', k);
                if this.inputPorts.isKey(portName{k})
                    portIDs(k) = this.inputPorts(portName{k});
                elseif this.outputPorts.isKey(portName{k})
                    portIDs(k) = this.outputPorts(portName{k});
                else
                    error('OBNNode:getPortInfo', 'Port name "%s" does not exist.', portName{k});
                end
            end
            
            info = repmat(struct('type', '', 'container', '', 'element', '', 'strict', false), 1, length(portIDs));
            for k = 1:length(portIDs)
                [info(k).type, info(k).container, info(k).element, info(k).strict] = ...
                    obnsim_yarp_('portInfo', this.id_, portIDs(k));
            end
        end
        
        % Returns the full path of a port given by its ID or name
        % For example a port named 'output1' on a node named 'mynode' will have
        % the full path '/mynode/output1'.
%         function s = fullPortPath(this, ID)
%             assert(isscalar(this));
%             if ischar(ID)
%                 ID = this.getPortID(ID);
%                 assert(ID > 0, 'Invalid port''s name.');
%             else
%                 assert(this.isValidID(ID), 'Invalid ID number.');
%             end
%             
%             s = Yarp_('yarpName', this.id_, ID);
%         end
       

        function addCallback(this, func, evtype, varargin)
            % Register a callback for an event.
            %       addCallback(this, func, evtype, ...)
            %
            % evtype is a non-empty string specifying the event type.
            % func is the function handle of the callback.
            %
            % The following events (evtype) are supported:
            % - 'Y' : UPDATE_Y event; the next argument must be the ID of
            %   the update type (between 0 and MaxUpdateID), the rest are
            %   custom arguments.
            % - 'X' : similar to 'Y' but for UPDATE_X event
            % - 'INIT' : SIM_INIT event when the simulation starts; the
            %   callback should set the initial states and initial outputs.
            % - 'TERM' : SIM_TERM event when the simulation terminates.
            
            narginchk(3, inf);
            assert(isscalar(this));
            assert(isa(func, 'function_handle'), 'func must be a function handle.');
            assert(ischar(evtype) && ~isempty(evtype), 'Event type must be a non-empty string.');
            
            switch upper(evtype)
                case {'Y', 'X'}
                    assert(length(varargin) >= 1, 'The update type ID must be provided.');
                    id = varargin{1};
                    assert(id >= 0 && id <= this.MaxUpdateID && floor(id) == id,...
                        'The update type ID must be an integer between 0 and %d', this.MaxUpdateID);
                    if strcmpi(evtype, 'Y')
                        tmp = this.callbacks_update_y;
                    else
                        tmp = this.callbacks_update_x;
                    end
                    
                    % Find the update type in the list already
                    found = find([tmp.id] == id);
                    if ~isempty(found)
                        % Remove it from the list
                        tmp(found) = [];
                    end
                    
                    % Add the new callback to the end
                    if strcmpi(evtype, 'Y')
                        this.callbacks_update_y = [tmp, struct('id', id, 'func', func, 'args', {varargin(2:end)})];
                    else
                        this.callbacks_update_x = [tmp, struct('id', id, 'func', func, 'args', {varargin(2:end)})];
                    end
                    
                case 'INIT'
                    this.callback_init = struct('func', func, 'args', {varargin});
                    
                case 'TERM'
                    this.callback_term = struct('func', func, 'args', {varargin});
                                        
                otherwise
                    error('OBNNode:addCallback', 'Unrecognized event type: %s.', evtype);
            end
        end
        
        
        function r = requestFutureUpdate(this, futureT, updates, timeout)
            %   r = requestFutureUpdate(this, futureT, update, timeout)
            % requests the SMN an irregular future event at time 'futureT'
            % with updates specified in 'updates'. This function blocks the
            % execution until it receives a response from the SMN for the
            % request or until the given timeout. If timeout <= 0, no
            % timeout can occur and this function can wait indefinitely.
            %
            % futureT must be an integer that specifies a time instant in
            %   the future, that is futureT > currentSimTime; simulation
            %   time is defined as the number of microseconds from the
            %   start of the simulation; the request is invalid if futureT
            %   is not in the future.
            %
            % updates must be a non-empty vector of indices of the desired
            %   updates. Its entries must be integers between 0 and
            %   MaxUpdateID.
            %
            % r is the result of the request; its value can be:
            %   -2 if the request is invalid (e.g. futureT is invalid).
            %   -1 if timeout occurred (request failed).
            %   0  if the request was successful (accepted).
            %   >0 if the request was rejected (failed) by the SMN; see the
            %       OBN design document for details.
            
            narginchk(3, 4);
            assert(isscalar(this));
            assert(isvector(updates) && isnumeric(updates) && ...
                all(updates >= 0 & updates <= this.MaxUpdateID) && ...
                all(updates == floor(updates)),...
                'updates must specify the indices of the desired updates.');
            
            if nargin < 4 || isempty(timeout)
                timeout = -1.0;
            else
                assert(isscalar(timeout) && isnumeric(timeout), ...
                    'Timeout must be a real number in seconds, or non-positive for no timeout.');
            end
            
            % Construct the update mask
            mask = uint64(0);
            for k = 1:length(updates)
                mask = bitset(mask, updates(k)+1);
            end
            
            % Call MEX with futureT converted to int64
            r = obnsim_yarp_('futureUpdate', this.id_, int64(futureT), mask, timeout);
        end
        
        
        function b = runSimulation(this, timeout)
            % The main function to run the simulation of the node.
            % An optional timeout can be given, which will set a timeout
            % for the node to wait for messages from the SMN/GC.
            % If a timeout occurs, the simulation of this node will stop
            % immediately by calling stop
            %
            % Return false if the execution is timed out; otherwise return
            % true.
            narginchk(1, 2);
            assert(isscalar(this));
            if nargin < 2 || isempty(timeout)
                timeout = -1;
            else
                assert(isscalar(timeout) && isnumeric(timeout), ...
                    'Timeout must be a real number in seconds, or non-positive for no timeout.');
            end
            
            % If the node has error, should not run it
            if this.hasError()
                error('OBNNode:runSimulation', 'Node currently has error and cannot run; please clear the error first by calling stopSimulation.');
            end
            
            % If the node is running --> may be not right, give a warning
            if this.isRunning()
                warning('OBNNode:runSimulation', 'Node is currently running; may be an error but we will run it anyway.');
            end
            
            status = 0;
            b = true;
            while status == 0
                [status, evtype, evargs] = obnsim_yarp_('runStep', this.id_, timeout);
                switch status
                    case 0  % Got an event
                        switch upper(evtype)
                            case 'Y'
                                this.onUpdateY(evargs);
                            case 'X'
                                this.onUpdateX(evargs);
                            case 'INIT'
                                this.onSimInit();
                            case 'TERM'
                                this.onSimTerm();
                            otherwise
                                error('OBNNode:runSimulation', 'Internal error: Unknown event type %s returned from MEX.', evtype);
                        end
                        
                    case 1  % Timeout
                        this.stopSimulation();  % stop the simulation immediately
                        b = false;
                        
                    case 2  % Stop properly
                        disp('Simulation has stopped properly.');
                        
                    case 3  % Stop with error
                        disp('Simulation has stopped due to an error.');
                        
                    otherwise
                        error('OBNNode:runSimulation', 'Internal error: Unknown running state returned from MEX.');
                end
            end
        end
        
        function b = hasError(this)
            % Test if the node currently has an error (ERROR state)
            assert(isscalar(this));
            b = obnsim_yarp_('isNodeErr', this.id_);
        end
        
        function b = isRunning(this)
            % Test if the node is running (RUNNING state)
            assert(isscalar(this));
            b = obnsim_yarp_('isNodeRunning', this.id_);
        end
        
        function b = isStopped(this)
            % Test if the node is stopped (STOPPED state)
            assert(isscalar(this));
            b = obnsim_yarp_('isNodeStopped', this.id_);
        end        
                
        function stopSimulation(this)
            % If this node is running, request/notify the SMN to stop, then
            % terminate the current node's simulation regardless of whether
            % the request was accepted or not.  This method also clear any
            % error in the node.  It's guaranteed that after this method,
            % the node's state is STOPPED.
            assert(isscalar(this));
            obnsim_yarp_('stopSim', this.id_);
        end

        function requestStopSimulation(this)
            % If this node is running, requests the SMN/GC to stop the
            % simulation (by sending a request message to the SMN).
            % However, the SMN/GC may deny the request and continue the
            % simulation.  This method will not stop the current node, so
            % if the request is not accepted, the node will continue
            % running.
            assert(isscalar(this));
            obnsim_yarp_('requestStopSim', this.id_);
        end

        function t = currentSimTime(this)
            %   t = node.currentSimTime()
            %returns the current simulation time as an integer number of
            %micro-seconds from the beginning of the simulation (when
            %simulation time is 0).
            assert(isscalar(this));
            t = obnsim_yarp_('simTime', this.id_);
        end
        
%         % Wait until a new event is received and process it (one event only)
%         function wait(this)
%             assert(isscalar(this));
%             [type, id] = Yarp_('wait', this.id_);
%             this.processEvent(type, id);
%         end
%         
%         % Return true if there is an event and it is processed
%         % Return false if timeout
%         function b = waitTimeout(this, T)
%             assert(isscalar(this));
%             assert(isnumeric(T) && isscalar(T) && T > 0);
%             
%             [b, type, id] = Yarp_('waitTimeout', this.id_, T);
%             if b
%                 this.processEvent(type, id);
%             end
%         end
%         
%         % Similar to waitTimeout() but does not block; if there is no event
%         % pending, it will return immediately. It's like waitTimeout(0).
%         function b = check(this)
%             assert(isscalar(this));
%             [b, type, id] = Yarp_('check', this.id_);
%             if b
%                 this.processEvent(type, id);
%             end
%         end
%         
%         % Process all pending events until there is no event left (event queue
%         % is empty).
%         function processAll(this)
%             assert(isscalar(this));
%             while check(this)
%             end
%         end
%         
%         % Run an infinite process loop
%         % If T is given, it's a timeout which may break the loop
%         function runProcessLoop(this, T)
%             assert(isscalar(this));
%             if nargin > 1
%                 % With timeout
%                 while this.waitTimeout(T)
%                 end
%             else
%                 while true
%                     % timeout to process callbacks and user commands
%                     if (~this.waitTimeout(1))
%                         drawnow;
%                     end
%                 end
%             end
%         end
%         

    end
    
    properties (Access=private)
        id_     % pointer to the C++ node object
        inputPorts     % map nodes' names to their IDs
        outputPorts     % map nodes' names to their IDs
        
        % These are the callbacks for events receiving from the node object
        % Each callback may receive some arguments, and returns nothing.
        
        % For both UPDATE_X and UPDATE_Y, we have an ordered list of
        % callbacks, each for a unique update type. Each has an ID (the ID
        % of the update type, an integer from 0 to MAX of update ID), a
        % function handle 'func' to call, and optional custom arguments
        % 'args' as a cell array. The callback is called with
        %       func(args{:})
        callbacks_update_y = struct('id', {}, 'func', {}, 'args', {});
        callbacks_update_x = struct('id', {}, 'func', {}, 'args', {});
        
        % For other simple events, there is a single callback function with
        % optional custom arguments. The callback is called with
        %       func(args{:})
        callback_init = struct('func', {}, 'args', {});
        callback_term = struct('func', {}, 'args', {});
    end
    
    methods (Static)
        function [b, vs] = isValidIdentifier(s)
            %   [b, vs] = isValidIdentifier(s)
            % Check if a given string is a valid identifier (name)
            % Also tries to obtain a valid name from s and returns in vs
            
            if ~ischar(s)
                vs = '';
                b = false;
                return;
            end
            
            vs = strtrim(s);
            b = ~isempty(vs) && vs(1) ~= '_' &&...
                all(ismember(lower(vs), 'abcdefghijklmnopqrstuvwxyz0123456789_'));
        end
        
        function b = isValidID(ID)
            b = isnumeric(ID) && isscalar(ID) && ID >= 0 && floor(ID)==ID;
        end
    end
    
    methods (Access=protected)
%         function processEvent(this, type, id)
%             switch type
%                 case 0  % message read
%                     this.onDataRcvd(id);
%                 otherwise
%                     warning('YarpNode:unknown_event', 'An unknown event type was received.');
%             end
%             
%             % Run through the list of events
%             for k = 1:length(this.allEvents)
%                 check(this.allEvents(k), type, id);
%             end
%         end
        
        function [portName, elementType] = checkPortSettings(this, portName, containerType, elementType)
            % Check validity of a new physical input/output port. Used by
            % create*Port functions.
            [b, portName] = OBNNode.isValidIdentifier(portName);
            assert(b, 'Invalid port''s name.');
            assert(~this.inputPorts.isKey(portName) && ~this.outputPorts.isKey(portName),...
                'Port with given name already exists.');
            
            assert(ischar(containerType) && isscalar(containerType) && ...
                ismember(lower(containerType), 'svmb'), 'Invalid container type.');
            
            if containerType == 'b'
                elementType = 'logical';
            else
                assert(ischar(elementType) && ~isempty(elementType), 'Element type must be a valid string.');
                elementType = lower(elementType);
                assert(ismember(elementType, {'logical', 'double', 'int32', 'uint32', 'int64', 'uint64'}),...
                    'Invalid element type.');
            end
        end
    end
end
