function b = yarp_disconnect(fromPort, toPort)
%YARP_DISCONNECT Disconnect two ports.
%   b = yarp_disconnect(fromPort, toPort)
%
%where fromPort and toPort are full names of the two ports to disconnect.
%
%Returns the status of the request as true (success) or false (failed).
%
%This file is part of the openBuildNet simulation framework developed at
%EPFL.
%
%Authors: Truong X. Nghiem (xuan.nghiem@epfl.ch)

%Last update: 2015-05-18.

assert(ischar(fromPort) && ischar(toPort) && ~isempty(fromPort) && ~isempty(toPort));
b = obnsim_yarp_('disconnect', fromPort, toPort);

end
