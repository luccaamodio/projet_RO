include("resolution.jl"); n, centers = readInputFile("../data/teste_4x4.txt"); println(cplexSolve(n, centers))
