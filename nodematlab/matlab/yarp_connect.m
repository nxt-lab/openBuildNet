function b = yarp_connect(fromPort, toPort, varargin)
%YARP_CONNECT Connect two ports in a Yarp network.
%   b = yarp_connect(fromPort, toPort [, key-value pairs])
%
%where fromPort and toPort are full names of the two ports to connect.
%The optional key-value pairs can be used to specify options for the
%connection. Currently the following options are supported:
%   'carrier'   Name of the carrier, e.g. 'tcp', 'udp', 'mcast' (see Yarp
%               documents); default: 'tcp'
%   'quiet'     Whether the connection status should be printed to the
%               standard output; default: true
%
%Returns the status of the request as true (success) or false (failed).
%
%This file is part of the openBuildNet simulation framework developed at
%EPFL.
%
%Authors: Truong X. Nghiem (xuan.nghiem@epfl.ch)

%Last update: 2015-05-18.

assert(ischar(fromPort) && ischar(toPort) && ~isempty(fromPort) && ~isempty(toPort));
if isempty(varargin)
    % A simple connection request
    b = obnsim_yarp_('connect', fromPort, toPort);
else
    % With options
    p = inputParser;
    addParameter(p, 'carrier', 'tcp', @(x) ischar(x) && ~isempty(x));
    addParameter(p, 'quiet', true, @(x) islogical(x) && isscalar(x));
    parse(p, varargin{:});
    
    b = obnsim_yarp_('connectExt', fromPort, toPort, p.Results.carrier, p.Results.quiet);
end

end
