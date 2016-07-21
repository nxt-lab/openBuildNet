# ADMM tracking example
# Aggregator node in Julia
#
# This file is part of the openBuildNet simulation framework developed at EPFL.
#
# Authors: Truong X. Nghiem (xuan.nghiem@epfl.ch)

function print_help()
  helpstring ="""aggregator.jl <N>

  where <N> is the number of local nodes.
  The environment variable OBNMQTT can be set to the address of the MQTT server.
  If it is not defined, the localhost address tcp://localhost:1883 will be used."""
  println(helpstring)
end

if length(ARGS) < 1
  println("Not enough input arguments.")
  print_help()
  exit(1)
end

Nnode = tryparse(Int64, ARGS[1])
if isnull(Nnode) || Nnode.value < 1
  println("Invalid number of local nodes.")
  print_help()
  exit(2)
end
Nnode = Nnode.value

using openbuildnet

const WORKSPACE = "disttracking"
const MAIN_UPDATE = 0
const maxIter = 50
const epsPrimal = 1e-5 # primal convergence criterion
const epsDual = 1e-2 # dual convergence criterion

# parameters and internal variables
Nhorizon = 5
rho = 10000

totalSteps = 50
reference = (rand(totalSteps) - 0.5) * Nnode / 2

# The following variables should be initialized inside the INIT callback
lambda = zeros(Nhorizon,1)
err = zeros(Nhorizon,1)

refHorizon = zeros(Nhorizon)  # reference for the horizon

Gammas = fill(rho*eye(Nhorizon), Nnode)
deltas = fill(zeros(Nhorizon), Nnode)

Σ = eye(Nnode) + ones(Nnode, Nnode)   # Constant matrix to explicitly solve the primal update

step = 1
admm_events = falses(Nnode,1)
in_admm_iters = false     # true if and only if ADMM iterations are ongoing
iter = 0

# define and reset ADMM variables
function reset_admm_vars()
  global U, Z, Z_prev, MU
  U = zeros(Nhorizon,Nnode)
  Z = zeros(Nhorizon,Nnode)
  Z_prev = zeros(Nhorizon,Nnode)
  MU = zeros(Nhorizon,Nnode)
end

# Vector of input and output ports
InputsU = Vector{OBNInputAbstract}()
OutputsGamma = Vector{OBNOutputAbstract}()
OutputsDelta = Vector{OBNOutputAbstract}()

# Computation of one iteration at the master node
function iter_computation()
  global MU += (U - Z)

  # local reference update
  global Z_prev = Z
  global Z = (U + MU .+ (refHorizon - lambda)) / Σ
  global err = sum(Z,2) - refHorizon

  # global dual update
  global lambda += err

  # Write to the output port(s) to send values to the local nodes immediately.
  global Gammas, deltas
  for k = 1:Nnode
    # Gammas[k] = rho*eye(Nhorizon)
    deltas[k] = rho * (-Z[:,k] + MU[:,k])
    sendsync(OutputsGamma[k], Gammas[k])
    sendsync(OutputsDelta[k], deltas[k])
  end
end

# ADMM iteration function for an input port corresponding to local node #k
# It is called every time a local node sends U<i> values to the aggregator.
# This callback should perform the ADMM iteration whenever all U<i>'s have been received.
# It should also check the termination condition.
# The computation should only be performed if and only if in_admm_iters = true; and when it finishes, it must reset this variable to stop the ADMM iterations.
function admmiter(port, k)
  global in_admm_iters, admm_events, U, iter

  if !in_admm_iters
    return;
  end

  # Mark the port event
  if admm_events[k]
    warn("During ADMM iteration, values from port U$k have been received multiple times.")
  end
  admm_events[k] = true
  U[:,k] = get(port)    # save the values

  # If all have been received, perform the iteration computation
  if all(admm_events)
    iter += 1

    fill!(admm_events, false) # reset tracking

    # Compute residual
    residualRef = sum(Z,2) - refHorizon
    convPrimal = sqrt(sum((U-Z).^2) + sum(residualRef.^2))
    convDual = sqrt(rho^2 * sum((Z - Z_prev).^2))

    # check convergence
    if ((convPrimal <= epsPrimal && convDual <= epsDual) || iter >= maxIter)
      println("Stop ADMM at iteration $iter: primal = $convPrimal, dual = $convDual")
      iter = 0
      in_admm_iters = false   # Stop iterations
      reset_admm_vars()
    else
      iter_computation()
    end
  end
end


# Create the node
node = OBNNode("Aggregator", WORKSPACE, get(ENV, "OBNMQTT", "tcp://localhost:1883"))

# Create individual input(s) for each local node to receive values from that node, for example x1 to xN as vectors.
for k = 1:Nnode
  push!(InputsU, create_input(node, "U$k", Vector{Float64}))
  push!(OutputsGamma, create_output(node, "Gamma$k", Matrix{Float64}))
  push!(OutputsDelta, create_output(node, "delta$k", Vector{Float64}))

  # Add callback on the U port
  on_receive(admmiter, InputsU[k], InputsU[k], k)
end

on_init(node) do
  println("INIT")
  reset_admm_vars()
end

on_term(node) do
  println("TERMINATE at $(sim_time(node,:s))")
end


on_block_output(node, MAIN_UPDATE) do
  # the current time, 0 if this is the initial step
  println("Start ADMM at time $(sim_time(node,:s)) (s)")

  global refHorizon = reference[step:(step-1+Nhorizon)]
  global step += 1

  # Start ADMM
  global in_admm_iters = true

  # Tell local nodes to start iterations
  iter_computation()

  # Loop to process all port events until it finishes, or a long timeout of 60s.
  while in_admm_iters
    if !process_port_events(node, 60)
      in_admm_iters = false
      stop(node, true)
      error("Timeout error while waiting for local nodes to update.")
    end
  end
end

println("Ready to start simulation...")
status = runsafe(node, 30)
println("Final status: ", status)
writecsv("reference.txt", reference)  # write the reference to a CSV file
