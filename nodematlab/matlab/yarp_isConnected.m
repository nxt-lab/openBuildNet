function b = yarp_isConnected(fromPort, toPort)
%YARP_ISCONNECTED Check if a connection exists.
%   b = yarp_isConnected(fromPort, toPort)
%
%where fromPort and toPort are full names of the two ports to disconnect.
%
%Returns true if fromPort is connected to toPort; false otherwise.
%
%This file is part of the openBuildNet simulation framework developed at
%EPFL.
%
%Authors: Truong X. Nghiem (xuan.nghiem@epfl.ch)

%Last update: 2015-05-18.

assert(ischar(fromPort) && ischar(toPort) && ~isempty(fromPort) && ~isempty(toPort));
b = obnsim_yarp_('isConnected', fromPort, toPort);

end
