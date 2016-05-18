%% Build the MEX interface for MQTTNode

% Get the path to the directory containing this script
mypath = fileparts(mfilename('fullpath'));
assert(~isempty(mypath));

% Save the current folder
current_folder = pwd;

% Change the toolbox folder
cd(mypath);

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
mexargs = [mexargs '-output private/obnsim_mqtt_ CXXFLAGS=''$CXXFLAGS -std=c++11 -stdlib=libc++'' '];
mexargs = [mexargs '-DOBNNODE_COMM_MQTT '];
mexargs = [mexargs '-I/usr/local/include/ -I../../nodematlab/include ' ...
  '-I../../nodecpp/include -I../../include ' ...
  '-Iprivate -L/usr/local/lib/ '];
mexargs = [mexargs '-lpaho-mqtt3a -lprotobuf '];
mexargs = [mexargs '../../nodecpp/src/obnnode_mqttnode.cpp ../../nodecpp/src/obnnode_mqttport.cpp ../../nodecpp/src/obnnode_basic.cpp ../../include/obnsim_basic.cpp ' ...
  '../../nodematlab/src/obnsim_mqtt.cc private/obnsim_msg.pb.cc private/obnsim_io.pb.cc '];

% Call mex to compile
eval(mexargs);

% Change back to the working folder
cd(current_folder);
