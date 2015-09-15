% Create private directory
if ~exist('private', 'dir')
    mkdir('private');
end

% Call protoc to generate ProtoBuf files
system('/usr/local/bin/protoc -I=../../msg --cpp_out=private ../../msg/obnsim_msg.proto ../../msg/obnsim_io.proto');

if ~exist('private/obnsim_msg.pb.h', 'file') || ~exist('private/obnsim_io.pb.h', 'file')
    error('Could not run ProtoBuf compiler to generate message classes.');
end

% Call mex to compile
mex ../../nodecpp/src/obnnode_yarpnode.cpp ../../nodecpp/src/obnnode_yarpport.cpp ../../nodecpp/src/obnnode_basic.cpp ../../include/obnsim_basic.cpp ...
  ../../nodematlab/src/obnsim_yarp.cc private/obnsim_msg.pb.cc private/obnsim_io.pb.cc -output private/obnsim_yarp_  ...
  CXXFLAGS='$CXXFLAGS -std=c++11 -stdlib=libc++' ...
  -I/usr/local/include/ -I../../nodematlab/include ...
  -I../../nodecpp/include -I../../include  ...
  -Iprivate -L/usr/local/lib/  ...
  -lYARP_dev -lYARP_name -lYARP_init -lYARP_math -lYARP_sig -lYARP_OS -lprotobuf
