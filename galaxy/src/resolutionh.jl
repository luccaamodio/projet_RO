include("io.jl")

"""
Heuristically solve a Galaxy instance using Backtracking and Reachability (BFS)
"""
function heuristicSolve(n::Int64, centers::Vector{Tuple{Float64, Float64}})
    
    start_time = time()
    n_centers = length(centers)
    grid = zeros(Int64, n, n)
    
    #plant the roots
    for e in 1:n_centers
        cx, cy = centers[e]
        for l in 1:n
            for c in 1:n
                # Adjust root alignment for grid coordinates
                if abs(cx - (l-0.5)) <= 0.51 && abs(cy - (c-0.5)) <= 0.51
                    grid[l, c] = e
                end
            end
        end
    end
    

    #reachability checker (BFS)
    # Check if all filled cells can still reach their roots.
    # This acts like CPLEX's "Network Flow" to prune isolated islands early.
    function is_viable(current_grid)
        visited = zeros(Bool, n, n)
        queue_l = zeros(Int, n*n)
        queue_c = zeros(Int, n*n)
        
        for l in 1:n
            for c in 1:n
                e = current_grid[l, c]
                
                # only check cells that are already assigned to a galaxy
                if e != 0
                    cx, cy = centers[e]
                    
                    # if the cell is the root itself, it is obviously connected
                    if abs(cx - (l-0.5)) <= 0.51 && abs(cy - (c-0.5)) <= 0.51
                        continue
                    end
                    
                    # Breadth-First Search (BFS) to find the root
                    fill!(visited, false)
                    queue_l[1] = l
                    queue_c[1] = c
                    visited[l, c] = true
                    found_root = false
                    
                    head = 1
                    tail = 1
                    
                    while head <= tail
                        cl = queue_l[head]
                        cc = queue_c[head]
                        head += 1
                        
                        # Root reached successfully
                        if abs(cx - (cl-0.5)) <= 0.51 && abs(cy - (cc-0.5)) <= 0.51
                            found_root = true
                            break
                        end
                        
                        # Search the 4 neighbors
                        for (dl, dc) in [(-1,0), (1,0), (0,-1), (0,1)]
                            nl, nc = cl + dl, cc + dc
                            
                            # Keep inside grid limits
                            if nl >= 1 && nl <= n && nc >= 1 && nc <= n
                                # The path can only travel through empty cells or the same galaxy
                                if !visited[nl, nc] && (current_grid[nl, nc] == 0 || current_grid[nl, nc] == e)
                                    visited[nl, nc] = true
                                    tail += 1
                                    queue_l[tail] = nl
                                    queue_c[tail] = nc
                                end
                            end
                        end
                    end
                    
                    # If a filled cell cannot reach its root, this path is a dead end
                    if !found_root
                        return false
                    end
                end
            end
        end
        return true
    end
    
    #backtracking engine
    # Internal recursive function
    function backtrack!(current_grid)
        
        # Abort the search if the 10-second limit is exceeded
        if time() - start_time > 10.0
            return false 
        end
        
        # find the first empty cell in the grid
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
        
        # If no empty cell is found, the grid is completely solved
        if vazia_l == -1
            return true
        end
        
        # try to assign this empty cell to a galaxy
        for e in 1:n_centers
            cx, cy = centers[e]
            
            # calculate the symmetric cell (mirror position)
            lp = round(Int, 2.0 * cx - vazia_l + 1)
            cp = round(Int, 2.0 * cy - vazia_c + 1)
            
            # validation: Mirror must be inside the grid limits
            if lp >= 1 && lp <= n && cp >= 1 && cp <= n
                
                # Mirror must be empty or already belong to this same galaxy
                if current_grid[lp, cp] == 0 || current_grid[lp, cp] == e
                    
                    modificou_espelho = (current_grid[lp, cp] == 0)
                    
                    # Make the move (paint both cells)
                    current_grid[vazia_l, vazia_c] = e
                    current_grid[lp, cp] = e
                    
                    # check viability (reachability to the root)
                    if is_viable(current_grid)
                        # Move forward
                        if backtrack!(current_grid)
                            return true 
                        end
                    end
                    
                    # Backtrack: Undo the move if it failed
                    current_grid[vazia_l, vazia_c] = 0
                    if modificou_espelho
                        current_grid[lp, cp] = 0
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

    dataFolder = "./galaxy/data/"
    resFolder = "./galaxy/res/"
    
    heurFolder = resFolder * "heuristique/"
    if !isdir(resFolder)
        mkdir(resFolder)
    end
    if !isdir(heurFolder)
        mkdir(heurFolder)
    end
            
    for file in filter(x->occursin(".txt", x), readdir(dataFolder))  
        
        println("-- Resolution of ", file)
        n, centers = readInputFile(dataFolder * file)
        
        outputFile = heurFolder * file
        if !isfile(outputFile)
            
            fout = open(outputFile, "w")  
            
            # Run the heuristic
            isOptimal, resolutionTime, solution_matrix = heuristicSolve(n, centers)

            # Save the results
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