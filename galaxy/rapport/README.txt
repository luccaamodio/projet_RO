include("galaxy\\src\\io.jl")
include("galaxy\\src\\resolution.jl")
solveDataSet()

include("galaxy\\src\\io.jl")
include("galaxy\\src\\resolutionh.jl")
solveDataSet()

resultsArray("galaxy//rapport//resultsArray")
performanceDiagram("galaxy//rapport//performanceDiagram.pdf")
