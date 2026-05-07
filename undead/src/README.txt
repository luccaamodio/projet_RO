Resolver uma instancia

include("undead\\src\\io.jl")
include("undead\\src\\resolution.jl")
t, views = readInputFile("undead\\data\\instance_t16_d0.1_1.txt")
cplexSolve(t, views)

Resolver todas as instancias

include("undead\\src\\generation.jl")
include("undead\\src\\io.jl")
include("undead\\src\\resolution.jl")
generateDataSet()
solveDataSet()
resultsArray("undead//rapport//resultsArray")
performanceDiagram("undead//rapport//performanceDiagram.pdf")