# This file contains methods to solve a Galaxy instance using CPLEX
using CPLEX
using JuMP
import MathOptInterface as MOI

include("io.jl")

"""
Solve a Galaxy instance with CPLEX using Network Flow constraints
"""
function cplexSolve(n::Int64, centers::Vector{Tuple{Float64, Float64}})

    # Create the model
    m = Model(CPLEX.Optimizer)
    
    # Disable excessive CPLEX output (optional)
    set_optimizer_attribute(m, "CPX_PARAM_SCRIND", 0)

    n_centers = length(centers)
    M = n * n # Maximum flow (total grid size)

    # 1. Variables
    # x[e, l, c]: 1 if cell (l,c) belongs to galaxy 'e'
    @variable(m, x[1:n_centers, 1:n, 1:n], Bin)
    
    # f[e, l, c, dir]: Continuous flow of water leaving cell (l,c) of galaxy 'e'
    # Directions: 1=Up, 2=Down, 3=Left, 4=Right
    @variable(m, f[1:n_centers, 1:n, 1:n, 1:4] >= 0)

    # 2. Linear constraints
    for l in 1:n
        for c in 1:n
            # Each cell must belong to exactly one galaxy
            @constraint(m, sum(x[e, l, c] for e in 1:n_centers) == 1)
        end
    end

    for e in 1:n_centers
        cx, cy = centers[e]
        for l in 1:n
            for c in 1:n
                # Diametrically opposite cell relative to the center (cx,cy)
                lp = round(Int, 2.0 * cx - l + 1) # Adds 1 because of the coordinates of the squares
                cp = round(Int, 2.0 * cy - c + 1) # Adds 1 because of the coordinates of the squares
                
                if lp < 1 || lp > n || cp < 1 || cp > n
                    # If the symmetric cell is outside the grid, x = 0, to boost speed
                    @constraint(m, x[e, l, c] == 0)
                elseif (l, c) < (lp, cp)
                    # Symmetry - The cell and its opposite must have the same value
                    @constraint(m, x[e, l, c] - x[e, lp, cp] == 0)
                end
            end
        end
    end

    
    # 3. Connectivity constraints (NETWORK FLOW)
    for e in 1:n_centers
        cx, cy = centers[e]
        
        for l in 1:n
            for c in 1:n
                
                # Outflow Capacity: Water only flows if the cell belongs to the galaxy
                @constraint(m, f[e, l, c, 1] <= M * x[e, l, c])
                @constraint(m, f[e, l, c, 2] <= M * x[e, l, c])
                @constraint(m, f[e, l, c, 3] <= M * x[e, l, c])
                @constraint(m, f[e, l, c, 4] <= M * x[e, l, c])

                # Inflow Capacity: Water only flows to neighbors that also belong to the galaxy
                if l > 1 @constraint(m, f[e, l, c, 1] <= M * x[e, l-1, c]) else @constraint(m, f[e, l, c, 1] == 0) end # Up
                if l < n @constraint(m, f[e, l, c, 2] <= M * x[e, l+1, c]) else @constraint(m, f[e, l, c, 2] == 0) end # Down
                if c > 1 @constraint(m, f[e, l, c, 3] <= M * x[e, l, c-1]) else @constraint(m, f[e, l, c, 3] == 0) end # Left
                if c < n @constraint(m, f[e, l, c, 4] <= M * x[e, l, c+1]) else @constraint(m, f[e, l, c, 4] == 0) end # Right

                # Checks if the center point is "touching" this cell (it acts as the water source)
                is_root = abs(cx - (l-0.5)) <= 0.51 && abs(cy - (c-0.5)) <= 0.51

                if is_root
                    # The roots always belong to the galaxy
                    @constraint(m, x[e, l, c] == 1)
                else
                    # Flow balance (Inflow - Outflow = 1 if it belongs to the galaxy)
                    inflow = AffExpr(0.0)
                    if l < n add_to_expression!(inflow, f[e, l+1, c, 1]) end # Comes from below
                    if l > 1 add_to_expression!(inflow, f[e, l-1, c, 2]) end # Comes from above
                    if c < n add_to_expression!(inflow, f[e, l, c+1, 3]) end # Comes from the right
                    if c > 1 add_to_expression!(inflow, f[e, l, c-1, 4]) end # Comes from the left
                    
                    outflow = f[e, l, c, 1] + f[e, l, c, 2] + f[e, l, c, 3] + f[e, l, c, 4]
                    
                    @constraint(m, inflow - outflow == x[e, l, c])
                end
            end
        end
    end

    # Start a chronometer
    start = time()

    # Solve the model
    optimize!(m)

    # Get the solver status
    status = JuMP.primal_status(m) == MOI.FEASIBLE_POINT
    resTime = time() - start

    # Build the solution matrix to be saved (if a solution exists)
    solution_matrix = zeros(Int64, n, n)
    if status
        for l in 1:n
            for c in 1:n
                for e in 1:n_centers
                    if value(x[e, l, c]) > 0.5
                        solution_matrix[l, c] = e
                    end
                end
            end
        end
    end

    # Return:
    # 1 - true if an optimum is found
    # 2 - the resolution time
    # 3 - the solved grid matrix
    return status, resTime, solution_matrix
end

"""
Solve all the instances contained in dataFolder through CPLEX

The results are written in resFolder

Remark: If an instance has previously been solved it will not be solved again
"""
function solveDataSet()

    dataFolder = "./galaxy/data/"
    resFolder = "./galaxy/res/"
    
    # Specific folder for CPLEX
    cplexFolder = resFolder * "cplex/"

    # Create folders if they do not exist
    if !isdir(resFolder)
        mkdir(resFolder)
    end
    if !isdir(cplexFolder)
        mkdir(cplexFolder)
    end
            
    # For each txt file in the data folder
    for file in filter(x->occursin(".txt", x), readdir(dataFolder))  
        
        println("-- Resolution of ", file)
        
        # Read the Galaxy instance data
        n, centers = readInputFile(dataFolder * file)
        
        outputFile = cplexFolder * file

        # If it hasn't been solved yet
        if !isfile(outputFile)
            
            fout = open(outputFile, "w")  
            
            # Run CPLEX!
            isOptimal, resolutionTime, solution_matrix = cplexSolve(n, centers)

            # Save the results in the txt file
            println(fout, "solveTime = ", resolutionTime) 
            println(fout, "isOptimal = ", isOptimal)
            
            # Write the solution matrix
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
            
            println("cplex optimal: ", isOptimal)
            println("cplex time: " * string(round(resolutionTime, sigdigits=2)) * "s\n")
        else
            println("Already solved! Skipping...")
        end         
    end 
end