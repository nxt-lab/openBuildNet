classdef OBNNode < matlab.mixin.Heterogeneous & handle
    % Class that implements an openBuildNet node in Matlab.
    %
    % This file is part of the openBuildNet simulation framework developed
    % at EPFL.
    %
    % Author: Truong X. Nghiem (xuan.nghiem@epfl.ch)
    
    % Last update: 2015-10-09

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
        function this = OBNNode(nodeName, ws, comm, server)
            %       obj = OBNNode(nodeName, ws = '', comm = 'yarp', server = '')
            % Create a new openBuildNet node in Matlab with a given name
            % and optionally a workspace name. The full name of the node in
            % the network will be /ws/nodeName
            %
            % comm is the communication used for this node; default is Yarp
            % Supported communication types: yarp, mqtt.
            %
            % server is the address of the server, used in certain
            % communication network (e.g. MQTT); default = '' which means
            % the default server (often a local server).
            
            narginchk(1, 4);
            [b, nodeName] = OBNNode.isValidNodeName(nodeName);
            assert(b, 'Invalid node''s name.');
            if nargin > 1 && ~isempty(ws)
                [b, ws] = OBNNode.isValidIdentifier(ws);
                assert(b, 'Invalid workspace name.');
            else
                ws = '';
            end
            
            this.nodeName = nodeName;
            
            if nargin > 2 && ~isempty(comm)
                comm = lower(comm);
            else
                comm = 'yarp';
            end
            
            if nargin < 4 || isempty(server)
                server = '';
            end
            
            switch comm
                case 'yarp'
                    this.obnnode_mexfunc_ = @obnsim_yarp_;
                    
                case 'mqtt'
                    this.obnnode_mexfunc_ = @obnsim_mqtt_;
                    
                otherwise
                    error('The communication %s is not supported.', comm);
            end
            
            % Try to access the MEX to make sure that it exists
            try
                this.MaxUpdateID = this.obnnode_mexfunc_('maxUpdateID');
            catch
                error('Could not call the MEX interface for the communication %s.', comm);
            end
            
            this.communication_ = comm;
            
            switch comm
                case 'yarp'
                    % Create a new node object and save its pointer in id_
                    this.id_  = obnsim_yarp_('nodeNew', nodeName, ws);
                    
                case 'mqtt'
                    % Create a new node object and save its pointer in id_
                    this.id_  = obnsim_mqtt_('nodeNew', nodeName, ws, server);
            end
            
            % Create the maps from ports' names to their IDs
            this.inputPorts = containers.Map('KeyType', 'char', 'ValueType', 'int32');
            this.outputPorts = containers.Map('KeyType', 'char', ...
                                              'ValueType', 'int32');
            
            % Create the map from ports' IDs to their callbacks
            this.callback_portrcvd = containers.Map('KeyType', 'int32', ...
                                                    'ValueType', 'any');
        end
        
        function delete(this)
            % disp('Delete');
            this.obnnode_mexfunc_('nodeDelete', this.id_);
        end
        
        function s = getCommunication(this)
            % Returns the communication for this node
            s = this.communication_;
        end
        
        function createInputPort(this, portName, containerType, elementType, varargin)
            %   createInputPort(this, portName,
            %                   containerType, elementType, ...)
            %
            % Create a new input port with a given name and type.
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
            
            id = this.obnnode_mexfunc_('createInput', this.id_, containerType, elementType, portName, varargin{:});
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
            
            id = this.obnnode_mexfunc_('createOutput', this.id_, containerType, elementType, portName);
            assert(id >= 0, 'Could not create and open the output port.');

            % save the port to the map
            this.outputPorts(portName) = id;
        end
        
        function d = input(this, portName)
            %    d = input(this, portName)
            % reads from an input port.
            % portName is the name of the input port.
            % If the port is non-strict, returns the current value of the
            % port.
            % If the port is strict, pops and returns the top / front value
            % of the port; after this call, that top / front value is
            % removed from the port.
            % In any case, the returned value is in an appropriate Matlab
            % type.
            % This function will cause an error if the port doesn't exist,
            % or if it's not an appropriate port type.
            %
            % Reading value from an input port can be expensive as it
            % involves calling a MEX function and transferring data from the
            % MEX to Matlab. Hence it's a good practice to read from an
            % input port once to a local variable to use, rather than
            % reading from the port repeatedly.
            
            narginchk(2, 2);
            assert(isscalar(this));
            % assert(this.isValidIdentifier(portName), 'Port name is invalid.');
            assert(this.inputPorts.isKey(portName), 'Input port does not exist.');
            
            d = this.obnnode_mexfunc_('readInput', this.id_, this.inputPorts(portName));
        end
        
        function b = inputPending(this, portName)
            %    b = inputPending(this, portName)
            % returns true if there is a pending value / message at the
            % given input port.
            narginchk(2, 2);
            assert(isscalar(this));
            assert(this.isValidIdentifier(portName), 'Port name is invalid.');
            assert(this.inputPorts.isKey(portName), 'Input port does not exist.');
            
            b = this.obnnode_mexfunc_('inputPending', this.id_, this.inputPorts(portName));
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
           % assert(this.isValidIdentifier(portName), 'Port name is invalid.');
           % assert(this.outputPorts.isKey(portName), 'Output port does not exist.');
           
           this.obnnode_mexfunc_('writeOutput', this.id_, this.outputPorts(portName), d);
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
           % to call sendSync() explicitly. However, for other events, e.g. port events, this
           % won't apply and therefore sendSync() needs to be used to send out the values.
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
               this.obnnode_mexfunc_('writeOutput', this.id_, this.outputPorts(portName), d);
           end
           this.obnnode_mexfunc_('sendSync', this.id_, this.outputPorts(portName));
        end
        
        function s = listInputs(this)
            % List all input ports either to the console or to an output
            % variable.
            narginchk(1, 1);
            assert(isscalar(this));

            ss = this.inputPorts.keys();
            if nargout == 0
                for k = 1:numel(ss)
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
                for k = 1:numel(ss)
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
            portIDs = zeros(numel(portName), 1);
            for k = 1:numel(portName)
                assert(this.isValidIdentifier(portName{k}), 'Port name #%d is invalid.', k);
                if this.inputPorts.isKey(portName{k})
                    portIDs(k) = this.inputPorts(portName{k});
                elseif this.outputPorts.isKey(portName{k})
                    portIDs(k) = this.outputPorts(portName{k});
                else
                    error('OBNNode:getPortInfo', 'Port name "%s" does not exist.', portName{k});
                end
            end
            
            info = repmat(struct('type', '', 'container', '', 'element', '', 'strict', false), 1, numel(portIDs));
            for k = 1:numel(portIDs)
                [info(k).type, info(k).container, info(k).element, info(k).strict] = ...
                    this.obnnode_mexfunc_('portInfo', this.id_, portIDs(k));
            end
        end
        
        function [result, msg] = connectFromPort(this, portName, srcPort)
            %   [result, msg] = connectFromPort(this, portName, srcPort)
            % Request to connect from a given srcPort to a port on this node.
            % - portName is a port's name (a string) on this node.
            % - srcPort is the address/fullname of the source port.
            %
            % Returns: result is 0 if successful, 1 if connection already
            % exists, < 0 if error (failed); msg is the message.
            
            narginchk(3, 3);
            assert(isscalar(this));
            assert(ischar(portName));
            assert(~isempty(portName), 'portName is empty.');
            assert(this.isValidIdentifier(portName), 'Port name is invalid.');
            assert(ischar(srcPort) && ~isempty(srcPort), 'Source port''s name must be a non-empty string.');
            
            if this.inputPorts.isKey(portName)
                portID = this.inputPorts(portName);
            elseif this.outputPorts.isKey(portName)
                portID = this.outputPorts(portName);
            else
                error('OBNNode:connectFromPort', 'Port name "%s" does not exist.', portName);
            end

            [result, msg] = this.obnnode_mexfunc_('connectPort', this.id_, portID, srcPort);
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
            %       addCallback(this, func, evtype, varargin)
            %
            % evtype is a non-empty string specifying the event type.
            % func is the function handle of the callback. It will be
            % called as func(custom arguments), where the custom arguments
            % are specified in varargin (see below).
            %
            % The following events (evtype) are supported:
            % - 'Y' : UPDATE_Y event; the next argument must be the ID of
            %   the update type (between 0 and MaxUpdateID), the rest are
            %   custom arguments.
            % - 'X' : similar to 'Y' but for UPDATE_X event
            % - 'INIT' : SIM_INIT event when the simulation starts; the
            %   callback should set the initial states and initial outputs.
            %   All of varargin are custom arguments.
            % - 'TERM' : SIM_TERM event when the simulation terminates. All
            %   of varargin are custom arguments.
            % - 'RCV' : event when an input port receives a new
            %   message / value. The next argument must be the name
            %   of the input port (a string). The rest are custom
            %   arguments.
            %
            % For events that don't support multiple callbacks
            % (e.g. INIT, TERM), when a callback is added, it will
            % replace any existing callback.
            %
            % IMPORTANT: only after UPDATE_Y and INIT events do the output ports automatically send
            % out their values (assigned by method output()). In callbacks for other events,
            % e.g. UPDATE_X or port events, output ports do not automatically send, therefore they
            % must be explicitly instructed to do so by calling sendSync().
            
            narginchk(3, inf);
            assert(isscalar(this));
            assert(isa(func, 'function_handle'), 'func must be a function handle.');
            assert(ischar(evtype) && ~isempty(evtype), 'Event type must be a non-empty string.');
            
            switch upper(evtype)
              case {'Y', 'X'}
                assert(numel(varargin) >= 1, 'The update type ID must be provided.');
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
                    
              case 'RCV'
                % The next argument must be a valid input port name
                % (already exists)
                assert(numel(varargin) >= 1 && ischar(varargin{1}) && ~isempty(varargin{1}),...
                       'The input port name must be provided.');
                assert(this.inputPorts.isKey(varargin{1}),...
                       'The given input port does not exist.');
                % Store the callback and arguments as a cell array
                this.callback_portrcvd(this.inputPorts(varargin{1})) = [{func}, ...
                                    varargin(2:end)];
                % Register the event at the input port
                if ~this.obnnode_mexfunc_('enableRcvEvent', this.id_, ...
                                          this.inputPorts(varargin{1}))
                    error('OBNNode:addCallback',...
                          'Error setting the RCV event callback.');
                end
              
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
            for k = 1:numel(updates)
                mask = bitset(mask, updates(k)+1);
            end
            
            % Call MEX with futureT converted to int64
            r = this.obnnode_mexfunc_('futureUpdate', this.id_, int64(futureT), mask, timeout);
        end

        function t = currentSimTime(this, timeunit)
            %   t = node.currentSimTime(timeunit)
            %returns the current simulation time as a real number in the
            %given time unit, from the beginning of the simulation (when
            %simulation time is 0).
            %Allowed time units are:
            % - 'second' or 's' [default if timeunit is omitted]
            % - 'minute' or 'm'
            % - 'hour' or 'h'
            % - 'millisecond' or 'ms'
            % - 'microsecond' or 'us'
            % - 'day' or 'd'
            assert(isscalar(this));
            
            scale = 1;  % scale of the time value returned from OBN
            
            if nargin < 2
                % default case
                obntu = 0;
            else
                switch lower(timeunit)
                    case {'s', 'second'}
                        obntu = 0;
                    case {'m', 'minute'}
                        obntu = 1;
                    case {'h', 'hour'} 
                        obntu = 2;
                    case {'ms', 'millisecond'}
                        obntu = -1;
                    case {'us', 'microsecond'}
                        obntu = -2;
                    case {'d', 'day'}
                        % OBN does not support this time unit, so we get
                        % the hour value and convert it to days
                        obntu = 2;
                        scale = 1/24;
                    otherwise
                        error('OBNNode:currentSimTime',...
                            'Unknown time unit %s', timeunit);
                end
            end
            
            t = this.obnnode_mexfunc_('simTime', this.id_, int16(obntu)) * scale;
        end
        
        function t = currentWallclockTime(this)
            %   t = node.currentWallclockTime()
            %returns the current wallclock time as a datetime value
            %(Matlab's data type to represent date and time).
            assert(isscalar(this));
            tposix = this.obnnode_mexfunc_('wallclock', this.id_);
            t = datetime(tposix, 'ConvertFrom', 'posixtime');
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
    
    methods (Sealed)
        function b = hasError(this)
            % Test if the node currently has an error (ERROR state)
            % If an object array is given, it returns an array of results.
            
            % assert(isscalar(this));
            if isscalar(this)
                b = this.obnnode_mexfunc_('isNodeErr', this.id_);
            else
                b = arrayfun(@(obj) obj.obnnode_mexfunc_('isNodeErr', obj.id_), this);
            end
        end
        
        function b = isRunning(this)
            % Test if the node is running (RUNNING state)
            % If an object array is given, it returns an array of results.
            
            %assert(isscalar(this));
            if isscalar(this)
                b = this.obnnode_mexfunc_('isNodeRunning', this.id_);
            else
                b = arrayfun(@(obj) obj.obnnode_mexfunc_('isNodeRunning', obj.id_), this);
            end
        end
        
        function b = isStopped(this)
            % Test if the node is stopped (STOPPED state)
            % If an object array is given, it returns an array of results.
            
            % assert(isscalar(this));
            if isscalar(this)
                b = this.obnnode_mexfunc_('isNodeStopped', this.id_);
            else
                b = arrayfun(@(obj) obj.obnnode_mexfunc_('isNodeStopped', obj.id_), this);
            end
        end        
                
        function stopSimulation(this)
            % If this node is running, request/notify the SMN to stop, then
            % terminate the current node's simulation regardless of whether
            % the request was accepted or not.  This method also clear any
            % error in the node.  It's guaranteed that after this method,
            % the node's state is STOPPED.
            %
            % If an object array is given, it will work for every node
            % object
            
            %assert(isscalar(this));
            for k = 1:numel(this)
                this(k).obnnode_mexfunc_('stopSim', this(k).id_);
            end
        end

        function requestStopSimulation(this)
            % If this node is running, requests the SMN/GC to stop the
            % simulation (by sending a request message to the SMN).
            % However, the SMN/GC may deny the request and continue the
            % simulation.  This method will not stop the current node, so
            % if the request is not accepted, the node will continue
            % running.
            %
            % If an object array is given, it will work for every node
            % object

            %assert(isscalar(this));
            for k = 1:numel(this)
                this(k).obnnode_mexfunc_('requestStopSim', this(k).id_);
            end
        end
        
        function b = processPortEvent(this, timeout)
        % Wait for and process the next port event (by executing its callback), up until a given
        % timeout (in seconds, can be non-positive for no timeout).
        % Returns true if an event has been processed; false otherwise (timeout).
        % This method can only be called on scalar objects.
            assert(isscalar(this));
            [b, evtype, portid] = this.obnnode_mexfunc_('portEvent', this.id_, timeout);
            if b
                if (evtype == 0)
                    assert(this.callback_portrcvd.isKey(portid),...
                           'Internal error: port id %d does not exist for RCV event.', portid);
                    thecallback = this.callback_portrcvd(portid);
                    feval(thecallback{:}); % Run the callback
                end
            end
        end
        
        function b = runSimulation(this, timeout, stopIfTimeout)
            % The main function to run the simulation of the node.
            %
            % An optional timeout (double value in seconds) can be given,
            % which will set a timeout for the node(s) to wait for messages
            % from the SMN/GC. If timeout is missing or <= 0, there is no
            % timeout and this function can run indefinitely (if the
            % simulation hangs).
            %
            % An optional stopIfTimeout [bool] can be given: if it's true
            % (default) then the simulation of these node(s) will stop
            % immediately if a timeout occurs by calling stopSimulation();
            % otherwise it will simply returns without stopping the
            % simulation (which can be resumed by calling this method
            % again).
            %
            % This method can take an array of OBNNode objects and
            % run them "in parallel" by switching between the nodes
            % quickly (in milliseconds). This allow multiple nodes
            % run in a single Matlab session, which is convenient
            % for development, with a slight tradeoff in
            % performance (due to waiting and switching time).
            %
            % Return, for each node, false if the execution is timed out;
            % otherwise return true.
            narginchk(1, 3);
            %assert(isscalar(this));
            nObjs = numel(this);  % Number of objects, because we can handle multiple node objects
            assert(nObjs > 0);
            
            if nargin < 2 || isempty(timeout)
                timeout = -1;
            else
                assert(isscalar(timeout) && isnumeric(timeout), ...
                    'Timeout must be a real number in seconds, or non-positive for no timeout.');
            end
            
            if nargin < 3 || isempty(stopIfTimeout)
                stopIfTimeout = true;
            end
            
            % If the node has error, should not run it
            if any(hasError(this))
                error('OBNNode:runSimulation', 'Node(s) currently has/have error and cannot run; please clear the error first by calling stopSimulation.');
            end
            
            % If the node is running --> may be not right, give a warning
            if any(isRunning(this))
                warning('OBNNode:runSimulation', 'Node(s) is/are currently running; may be an error but we will run it anyway.');
            end
            
            % To help Matlab process its own callbacks (e.g. for GUI, user
            % interruption...) we will call OBN MEX with a small timeout
            % value to return to Matlab and allow it to handle its events
            % and callbacks, then we go back to simulation. At the same
            % time, we keep track of the user-given timeout value, and do
            % stop the simulation if that timeout error occurs.
            
            status = zeros(nObjs,1);  % store the status values of the nodes
            nRemaining = nObjs;
            b = true(size(this));
            timeoutStart = tic;
            drawnowStart = timeoutStart;    % to allow drawnow() every now and then, not too often
            
            while nRemaining > 0
                if nRemaining < 2
                    tWaitfor = 1.0;
                else
                    tWaitfor = max(min(1.0/nRemaining, 0.005), 0.002);
                end
                for k = 1:nObjs
                    if status(k) ~= 0
                        continue;
                    end
                    
                    [status(k), evtype, evargs] = this(k).obnnode_mexfunc_('runStep', this(k).id_, tWaitfor);  % small timeout is used
                    switch status(k)
                        case 0  % Got an event
                            switch upper(evtype)
                                case 'Y'
                                    this(k).onUpdateY(evargs);
                                case 'X'
                                    this(k).onUpdateX(evargs);
                                case 'INIT'
                                    this(k).onSimInit();
                                case 'TERM'
                                    this(k).onSimTerm();
                                case 'RCV'
                                    assert(this(k).callback_portrcvd.isKey(evargs),...
                                           ['Internal error: input port id %d does not exist for RCV ' ...
                                            'event.'], evargs);
                                    thecallback = this(k).callback_portrcvd(evargs);
                                    feval(thecallback{:});
                                otherwise
                                    error('OBNNode:runSimulation', 'Internal error: Unknown event type %s returned from MEX for node %s.', evtype, this(k).nodeName);
                            end
                            timeoutStart = tic;     % reset the timer
                            
                        case 1  % Timeout
                            if toc(timeoutStart) > timeout
                                % Real timeout error occurred
                                if stopIfTimeout
                                    this(k).stopSimulation();  % stop the simulation immediately
                                end
                                b(k) = false;
                            else
                                % We can continue running the simulation
                                status(k) = 0;
                            end
                            
                        case 2  % Stop properly
                            disp(['Simulation of node ' this(k).nodeName ' has stopped properly.']);
                            
                        case 3  % Stop with error
                            disp(['Simulation of node ' this(k).nodeName ' has stopped due to an error.']);
                            
                        otherwise
                            error('OBNNode:runSimulation', 'Internal error: Unknown running state of node %s returned from MEX.', this(k).nodeName);
                    end
                    if status(k) ~= 0
                        nRemaining = nRemaining - 1;
                    end
                end
                % give Matlab a chance to update its GUI and process
                % callbacks, every 1 second
                if toc(drawnowStart) > 1
                    drawnow;
                    drawnowStart = tic;
                end
            end
        end
        
    end
    
    properties (Access=private)
        obnnode_mexfunc_    % The MEX function to call, according to the chosen communication
        communication_ = 'yarp';    % The chosen communication protocol
        id_     % pointer to the C++ node object
        inputPorts     % map ports' names to their IDs
        outputPorts     % map ports' names to their IDs
        nodeName  % Name of the node
        
        MaxUpdateID
        
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
        
        % Callbacks for input ports' message received events
        % map ports' names to  callback functions
        callback_portrcvd
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
        
        function [b, vs] = isValidNodeName(s)
            %   [b, vs] = isValidNodeName(s)
            % Check if a given string is a valid node name (cf. C++ code
            % obnsim_basic.cpp).
            % Also tries to obtain a valid name from s and returns in vs
            
            if ~ischar(s)
                vs = '';
                b = false;
                return;
            end
            
            vs = strtrim(s);
            b = ~isempty(vs) && vs(1) ~= '_' && vs(1) ~= '/' && vs(end) ~= '/' &&...
                all(ismember(lower(vs), 'abcdefghijklmnopqrstuvwxyz0123456789_/')) &&...
                isempty(strfind(vs, '//')) && isempty(strfind(vs, '/_'));
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
%             for k = 1:numel(this.allEvents)
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
