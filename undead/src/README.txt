include("undead\\src\\io.jl")
include("undead\\src\\resolution.jl")
t, views = readInputFile("undead\\data\\instance_t16_d0.1_1.txt")
cplexSolve(t, views)