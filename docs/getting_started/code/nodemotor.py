# Calling syntax: nodemotor.py [MQTT server address]
from __future__ import division, print_function
import numpy as np
import sys
from obnpy.obnnode import *

class Motor(OBNNode):
    """Node that implements a DC motor model."""
    def __init__(self, workspace, server):
        OBNNode.__init__(self, "motor", workspace, server)

        self.create_input("vol", "scalar", "double") # Create voltage input
        self.create_output("v", "scalar", "double")  # Create velocity output

        MAINBLOCK = 0                   # There is only one block in this node (the motor's dynamics)
        self.on_init(self.initCallback) # assign callback to initialize the node
        self.on_term(lambda: print("Motor node terminated.")) # simple callback for node's termination
        self.on_block_output(self.motorOutput, MAINBLOCK)     # callback to send output
        self.on_block_state(self.updateState, MAINBLOCK)      # callback to update the motor's states

        # Initialize the system's matrices
        self.A = np.array([(1.511, -0.5488), (1.0, 0.0)])
        self.C = np.array([(0.03294, 0.02697)])


    def initCallback(self):
        self.x = np.zeros(2) # Reset the state vector
        self.output_ports["v"].set(0.0) # Initialize the output (not necessary)
        print("At {} simulation started.".format(self.sim_time()))

    def motorOutput(self):
        v = np.dot(self.C, self.x)
        self.output_ports["v"].set(v[0]) # Set the output value
        
    
    def updateState(self):
        v = self.input_ports["vol"].get() # Get the input voltage
        self.x = np.dot(self.A, self.x)
        self.x[0] += 0.0625 * v



def main():
    if len(sys.argv) < 2:
        server = 'tcp://localhost:1883'
    else:
        server = sys.argv[1]

    node = Motor("test2", server)
    print("Ready to run the motor node; please start all other nodes ...")
    status = node.run(60)
    print("Simulation stopped with status = {}".format(status))


if __name__ == '__main__':
    main()
    
