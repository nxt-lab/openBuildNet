# Calling syntax: nodectrl.py [MQTT server address]
from __future__ import division, print_function
import numpy as np
import sys
import csv
from obnpy.obnnode import *

class Controller(OBNNode):
    """Node that implements a controller for a DC motor."""
    def __init__(self, workspace, server):
        OBNNode.__init__(self, "ctrl", workspace, server)

        self.create_input("v", "scalar", "double")  # Create velocity input
        self.create_input("sp", "scalar", "double") # Create setpoint input
        self.create_output("u", "scalar", "double") # Create command output

        MAINBLOCK = 0                   # There is only one block in this node
        self.on_init(self.initCallback) # assign callback to initialize the node
        self.on_term(self.termCallback) # simple callback for node's termination
        self.on_block_output(self.ctrlOutput, MAINBLOCK) # callback to send output
        self.on_block_state(self.updateState, MAINBLOCK) # callback to update the controller's states

        # Initialize the system's matrices
        self.A = np.array([(-0.82, 1.0, 0.82), (1.0 , 0.0, 0.0), (0.0 , 1.0, 0.0)])
        self.C = np.array([(12.62, -19.75, 7.625)])


    def initCallback(self):
        self.x = np.zeros(3) # Reset the state vector
        self.command = 0.0
        self.output_ports["u"].set(self.command) # Initialize the output (not necessary)
        self.dumpfile = open("controller.txt", "wb")
        self.dump = csv.writer(self.dumpfile, delimiter="\t")
        print("At {} simulation started.".format(self.sim_time()))

    def termCallback(self):
        self.dumpfile.close()
        print("Controller node terminated.")

    def ctrlOutput(self):
        self.command = np.dot(self.C, self.x)
        self.output_ports["u"].set(self.command[0])
        
    def updateState(self):
        sp = self.input_ports["sp"].get() # Get the setpoint
        v = self.input_ports["v"].get() # Get the velocity
        self.x = np.dot(self.A, self.x)
        self.x[0] += 32.0 * (sp - v)

        # At this point, the inputs are all up-to-date, so we can dump log data
        self.dump.writerow([self.sim_time(), sp, v] + self.command.tolist() + self.x.tolist())
        
def main():
    if len(sys.argv) < 2:
        server = 'tcp://localhost:1883'
    else:
        server = sys.argv[1]

    node = Controller("test2", server)
    print("Ready to run the controller node; please start all other nodes ...")
    status = node.run(60)
    print("Simulation stopped with status = {}".format(status))


if __name__ == '__main__':
    main()
    
