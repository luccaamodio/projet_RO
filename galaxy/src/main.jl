include("io.jl")
include("resolution.jl")
include("resolutionh.jl")

function main()
    # Exemple d'utilisation : résoudre le premier fichier de data
    dataFolder = "../data/"
    files = filter(x -> occursin(".txt", x), readdir(dataFolder))
    if length(files) == 0
        println("Aucun fichier de données trouvé dans ", dataFolder)
        return
    end

    file = files[1]
    println("Lecture de l'instance : ", file)
    n, centers = readInputFile(dataFolder * file)

    println("--- Résolution CPLEX ---")
    isOptimal, solveTime, solution = cplexSolve(n, centers)
    println("CPLEX optimal: ", isOptimal)
    println("Temps CPLEX: ", solveTime, "s")
    if isOptimal
        println("Solution CPLEX:")
        println(solution)
    end

    println("\n--- Résolution heuristique ---")
    isOptimalH, solveTimeH, solutionH = heuristicSolve(n, centers)
    println("Heuristique optimal: ", isOptimalH)
    println("Temps heuristique: ", solveTimeH, "s")
    if isOptimalH
        println("Solution heuristique:")
        println(solutionH)
    end
end

main()
