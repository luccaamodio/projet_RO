Para usar os displays:

include("undead//src//io.jl")
t, views = readInputFile("undead//data//instance_t4_d0.1_1.txt")
displayGrid(t, views)
include("undead//src//resolution.jl")
isOptimum, X, time = cplexSolve(t, views)
displaySolution(X, t, views)