# Node that implements a controller for a DC motor.
# Calling syntax: nodectrl.jl [MQTT server address]
using openbuildnet

# Process input arguments
mqttserver = length(ARGS)>0?String(ARGS[1]):"tcp://localhost:1883"

# Controller's states
x = zeros(3)
command = 0.0

# The parameters
A = [-0.82 1.0 0.82; 1.0 0.0 0.0; 0.0 1.0 0.0]
C = [12.62 -19.75 7.625]

# Create the node
workspace = "test2"
node = OBNNode("ctrl", workspace, mqttserver)

# Create inputs
u_v = create_input(node, "v", Float64)  # Create velocity input
u_sp = create_input(node, "sp", Float64) # Create setpoint input

# Create output
y_u = create_output(node, "u", Float64) # Create command output

# Create the CSV writer
dumpfile = open("controller.txt", "w")

# Create Callbacks
on_init(node) do
    set(y_u, command) # Initialize the output (not necessary)
    println("At ", sim_time(node), " simulation started.")
end

on_term(node) do
    close(dumpfile)  # Close the CSV file
    println("At ", sim_time(node), " simulation terminated.")
end

const MAINBLOCK = 0

on_block_output(node, MAINBLOCK) do
    global command
    command = C * x
    set(y_u, command[1])
end

on_block_state(node, MAINBLOCK) do
    # Get the inputs
    sp = get(u_sp)
    v = get(u_v)
    global x
    x = A * x
    x[1] += 32.0 * (sp - v)

    # Write to the CSV file
    write(dumpfile, join((sim_time(node), sp, v), "\t"), "\t",
          join(command, "\t"), "\t", join(x, "\t"), "\n")
end

println("Ready to run the controller node; please start all other nodes ...")
status = runsafe(node, 60)
println("Simulation stopped with status = ", status)
