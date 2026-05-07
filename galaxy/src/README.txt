Résoudre une instance Galaxy avec CPLEX

include("galaxy\\src\\io.jl")
include("galaxy\\src\\resolution.jl")
n, centers = readInputFile("galaxy\\data\\teste_4x4.txt")
cplexSolve(n, centers)

Résoudre une instance Galaxy avec l'heuristique

include("galaxy\\src\\io.jl")
include("galaxy\\src\\resolutionh.jl")
n, centers = readInputFile("galaxy\\data\\teste_4x4.txt")
heuristicSolve(n, centers)

Générer les instances

include("galaxy\\src\\generation.jl")
generateDataSet()

Résoudre toutes les instances de data avec CPLEX

include("galaxy\\src\\io.jl")
include("galaxy\\src\\resolution.jl")
solveDataSet()

Résoudre toutes les instances de data avec l'heuristique

include("galaxy\\src\\io.jl")
include("galaxy\\src\\resolutionh.jl")
solveDataSet()

Créer un diagramme de performance

include("galaxy\\src\\io.jl")
performanceDiagram("galaxy//rapport//performanceDiagram.pdf")

Créer un tableau de résultats en LaTeX

include("galaxy\\src\\io.jl")
resultsArray("galaxy//rapport//resultsArray")
