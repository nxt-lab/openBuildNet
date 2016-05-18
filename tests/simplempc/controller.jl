# The controller node
# This file is part of the openBuildNet framework developed at EPFL.
# (C) 2016 LA-EPFL
# Author: Truong X. Nghiem (truong.nghiem@gmail.com, xuan.nghiem@epfl.ch)

# MPC node
# See: http://users.isy.liu.se/johanl/yalmip/pmwiki.php?n=Examples.StandardMPC

using openbuildnet

# Model data
const A = [2. -1.;1. 0.2]
const B = [1. 0.]'
const x0 = [3., 1.];  # Initial state

# MPC data
nx = size(A,2); # Number of states
nu = size(B,2); # Number of inputs

# MPC data
Q = eye(nx);
R = 2.;
N = 7;

# Prepare the optimization model
using GLPKMathProgInterface
using Convex

function create_model()
  local u = Variable(nu, N);
  local x = Variable(nx, N);
  local x0v = Variable(nx);

  local constraints = [-1 <= u, u <= 1, -5 <= x, x <= 5];
  constraints += x[:,1] == A*x0v + B*u[:,1];
  objective = norm(Q*x0v,1) + norm(R*u[:,1],1);
  for k = 2:N
    objective = objective + norm(Q*x[:,k-1],1) + norm(R*u[:,k],1);
    constraints += x[:,k] == A*x[:,k-1] + B*u[:,k];
  end

  controller = minimize(objective, constraints);

  # returns a tuple of the controller, the parameter, and the solution
  (controller, x0v, u)
end

tic()
controller, x0v, u = create_model()
toc()

# Create the node
node = OBNNode("controller", "simplempc")

# Create the ports
input_x = create_input(node, "x", Vector{Float64})  # feedback
output_u = create_output(node, "u", Vector{Float64}) # control values

# Create Callbacks
on_init(node) do
  println("INIT")
end

on_term(node) do
  println("TERMINATE")
end

const MAINBLOCK = 0

US = Vector{Vector{Float64}}()  # To store the control values

on_block_output(node, MAINBLOCK) do
  global controller, x0v, u
  fix!(x0v, get(input_x))
  tic()
  solve!(controller, GLPKSolverLP())
  toc()
  uk = u.value[:,1]
  push!(US, uk)
  set(output_u, uk)
end


println("Ready to start simulation...")
status = runsafe(node, 20)
println("Final status: ", status)

# Plot the results
if status == 2
  using PyPlot
  USmat = hcat(US...)' # convert to matrix
  plot(USmat[:,1])
  show()
end
