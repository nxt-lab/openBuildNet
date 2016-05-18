# The plant node
# This file is part of the openBuildNet framework developed at EPFL.
# (C) 2016 LA-EPFL
# Author: Truong X. Nghiem (truong.nghiem@gmail.com, xuan.nghiem@epfl.ch)

# See: http://users.isy.liu.se/johanl/yalmip/pmwiki.php?n=Examples.StandardMPC
# The plant model:
# x(t+1) = A*x(t) + B*u(t)
# y(t) = x(t)

using openbuildnet

# Model data
const A = [2. -1.;1. 0.2]
const B = [1. 0.]'
const x0 = [3., 1.];  # Initial state

x = x0  # The plant's state
xs = Vector{typeof(x)}()  # To store the state trajectory
push!(xs, x)

# Create the node
node = OBNNode("plant", "simplempc")

# Create the ports
u = create_input(node, "u", Vector{Float64})
y = create_output(node, "y", Vector{Float64})

# Create Callbacks
on_init(node) do
  println("INIT")
end

on_term(node) do
  println("TERMINATE")
end

const MAINBLOCK = 0

on_block_output(node, MAINBLOCK) do
  set(y, x)
end

on_block_state(node, MAINBLOCK) do
  global x
  x = A*x + B*get(u)
  push!(xs, x)  # save the state
end

println("Ready to start simulation...")
status = runsafe(node, 20)
println("Final status: ", status)

# Plot the results
if status == 2
  using PyPlot
  xsmat = hcat(xs...)' # convert to matrix
  plot(xsmat[:,1], xsmat[:,2])
  show()
end
