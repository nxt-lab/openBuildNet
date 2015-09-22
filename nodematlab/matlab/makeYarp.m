% Select the communication frameworks you want to support
useYarp = true;
useMQTT = true;

% Create private directory
if ~exist('private', 'dir')
    mkdir('private');
end

% Call protoc to generate ProtoBuf files
system('/usr/local/bin/protoc -I=../../msg --cpp_out=private ../../msg/obnsim_msg.proto ../../msg/obnsim_io.proto');

if ~exist('private/obnsim_msg.pb.h', 'file') || ~exist('private/obnsim_io.pb.h', 'file')
    error('Could not run ProtoBuf compiler to generate message classes.');
end

mexargs = 'mex ';
mexargs = [mexargs '-output private/obnsim_yarp_ CXXFLAGS=''$CXXFLAGS -std=c++11 -stdlib=libc++'' '];
if useYarp
    mexargs = [mexargs '-DOBNNODE_COMM_YARP '];
end
if useMQTT
    mexargs = [mexargs '-DOBNNODE_COMM_MQTT '];
end
mexargs = [mexargs '-I/usr/local/include/ -I../../nodematlab/include ' ...
  '-I../../nodecpp/include -I../../include ' ...
  '-Iprivate -L/usr/local/lib/ '];
mexargs = [mexargs '-lYARP_dev -lYARP_name -lYARP_init -lYARP_sig -lYARP_OS -lprotobuf '];
mexargs = [mexargs '../../nodecpp/src/obnnode_yarpnode.cpp ../../nodecpp/src/obnnode_yarpport.cpp ../../nodecpp/src/obnnode_basic.cpp ../../include/obnsim_basic.cpp ' ...
  '../../nodematlab/src/obnsim_yarp.cc private/obnsim_msg.pb.cc private/obnsim_io.pb.cc '];

% Call mex to compile
eval(mexargs);
