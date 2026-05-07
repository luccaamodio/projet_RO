include("io.jl"); include("resolutionh.jl"); n, centers = readInputFile("../data/teste_4x4.txt"); println(heuristicSolve(n, centers))
