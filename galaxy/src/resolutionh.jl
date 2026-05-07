# Only the IO library, no CPLEX!
include("io.jl")

"""
Heuristically solve a Galaxy instance using Backtracking and Symmetric Flood Fill
"""
function heuristicSolve(n::Int64, centers::Vector{Tuple{Float64, Float64}})
    
    start_time = time()
    n_centers = length(centers)
    
    # Initialize empty grid (0 means cell without galaxy)
    grid = zeros(Int64, n, n)
    
    # Plant the roots (Cells touched by the center point)
    for e in 1:n_centers
        cx, cy = centers[e]
        for l in 1:n
            for c in 1:n
                if abs(cx - l) <= 0.51 && abs(cy - c) <= 0.51
                    grid[l, c] = e
                end
            end
        end
    end
    
    # Internal recursive function (The "engine" of the heuristic)
    function backtrack!(current_grid)
        # Find the first empty cell in the grid
        vazia_l, vazia_c = -1, -1
        for l in 1:n
            for c in 1:n
                if current_grid[l, c] == 0
                    vazia_l, vazia_c = l, c
                    break
                end
            end
            if vazia_l != -1 break end
        end
        
        # If no empty cell is found, the grid is full! (SUCCESS)
        if vazia_l == -1
            return true
        end
        
        # Try to assign this cell to a galaxy
        for e in 1:n_centers
            cx, cy = centers[e]
            
            # Cell reflection
            lp = round(Int, 2.0 * cx - vazia_l)
            cp = round(Int, 2.0 * cy - vazia_c)
            
            # Validations
            if lp >= 1 && lp <= n && cp >= 1 && cp <= n
                if current_grid[lp, cp] == 0 || current_grid[lp, cp] == e
                    
                    # Connectivity: must be a neighbor to someone from the same galaxy
                    is_vizinha = false
                    vizinhos = [(vazia_l-1, vazia_c), (vazia_l+1, vazia_c), 
                                (vazia_l, vazia_c-1), (vazia_l, vazia_c+1)]
                    for (vl, vc) in vizinhos
                        if vl >= 1 && vl <= n && vc >= 1 && vc <= n
                            if current_grid[vl, vc] == e
                                is_vizinha = true
                                break
                            end
                        end
                    end
                    
                    if is_vizinha
                        modificou_espelho = (current_grid[lp, cp] == 0)
                        
                        current_grid[vazia_l, vazia_c] = e
                        current_grid[lp, cp] = e
                        
                        if backtrack!(current_grid)
                            return true 
                        end
                        
                        # It failed, undo the move (Backtrack)
                        current_grid[vazia_l, vazia_c] = 0
                        if modificou_espelho
                            current_grid[lp, cp] = 0
                        end
                    end
                end
            end
        end
        return false
    end
    
    is_optimal = backtrack!(grid)
    resolution_time = time() - start_time
    
    return is_optimal, resolution_time, grid
end

"""
Solve all instances in the data folder using only the Heuristic
"""
function solveDataSet()

    dataFolder = "../data/"
    resFolder = "../res/"
    
    # Specific folder for the heuristic
    heurFolder = resFolder * "heuristique/"

    # Create folders if they do not exist
    if !isdir(resFolder)
        mkdir(resFolder)
    end
    if !isdir(heurFolder)
        mkdir(heurFolder)
    end
            
    # For each txt file in the data folder
    for file in filter(x->occursin(".txt", x), readdir(dataFolder))  
        
        println("-- Resolution of ", file)
        n, centers = readInputFile(dataFolder * file)
        
        outputFile = heurFolder * file

        # If it hasn't been solved yet
        if !isfile(outputFile)
            
            fout = open(outputFile, "w")  
            
            # Run the heuristic!
            isOptimal, resolutionTime, solution_matrix = heuristicSolve(n, centers)

            # Save the results in the txt file
            println(fout, "solveTime = ", resolutionTime) 
            println(fout, "isOptimal = ", isOptimal)
            
            if isOptimal && solution_matrix !== nothing
                println(fout, "\n# Solved Matrix:")
                for l in 1:n
                    for c in 1:n
                        print(fout, rpad(solution_matrix[l, c], 4))
                    end
                    println(fout, "")
                end
            end 

            close(fout)
            
            println("heuristique optimal: ", isOptimal)
            println("heuristique time: " * string(round(resolutionTime, sigdigits=2)) * "s\n")
        else
            println("Already solved! Skipping...")
        end         
    end 
end