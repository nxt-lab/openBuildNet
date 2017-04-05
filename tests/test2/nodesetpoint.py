# Calling syntax: nodesetpoint.py [MQTT server address]
from __future__ import division, print_function
import numpy as np
import sys
from obnpy.obnnode import *

node = None                     # This will be the node object
setpoint = 0.0                  # The current setpoint value

def initNode():                 # Init callback
    global setpoint
    setpoint = 0.0              # Reset the setpoint value

def outputSetpoint():           # Change and send the setpoint
    global setpoint, node
    setpoint = np.random.randint(-100, 101) / 10.0
    node.output_ports["sp"].set(setpoint)

def main():
    if len(sys.argv) < 2:
        server = 'tcp://localhost:1883'
    else:
        server = sys.argv[1]

    global node
    node = OBNNode("sp", "test2", server) # Create a node

    node.create_output("sp", "scalar", "double") # The setpoint output

    node.on_init(initNode)
    node.on_term(lambda: print("Setpoint node terminated."))

    MAINBLOCK = 0
    node.on_block_output(outputSetpoint, MAINBLOCK)

    print("Ready to run the setpoint node; please start all other nodes ...")
    status = node.run(60)
    print("Simulation stopped with status = {}".format(status))


if __name__ == '__main__':
    main()

