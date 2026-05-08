# This file contains methods to solve a Galaxy instance using CPLEX with callback-based connectivity constraints
using CPLEX
using JuMP
import MathOptInterface as MOI

include("io.jl")

"""
Solve a Galaxy instance with CPLEX using lazy connectivity cuts
"""
function cplexSolveWithCallback(n::Int64, centers::Vector{Tuple{Float64, Float64}})

    # =========================
    # Create model
    # =========================
    m = Model(CPLEX.Optimizer)
    
    # Disable excessive CPLEX console output
    set_optimizer_attribute(m, "CPX_PARAM_SCRIND", 0)

    n_centers = length(centers)

    # =========================
    # Variables
    # =========================
    # x[e,l,c] = 1 if cell (l,c) belongs to galaxy e
    @variable(m, x[1:n_centers, 1:n, 1:n], Bin)

    # =========================
    # Basic constraints
    # =========================
    # Each cell belongs to exactly one galaxy
    for l in 1:n
        for c in 1:n
            @constraint(m, sum(x[e,l,c] for e in 1:n_centers) == 1)
        end
    end

    # =========================
    # Symmetry constraints
    # =========================
    for e in 1:n_centers
        cx, cy = centers[e]
        for l in 1:n
            for c in 1:n
                # Calculate the symmetric mirror cell
                lp = round(Int, 2.0 * cx - l + 1)
                cp = round(Int, 2.0 * cy - c + 1)

                # If the symmetric cell is outside the grid, this cell cannot belong to the galaxy
                if lp < 1 || lp > n || cp < 1 || cp > n
                    @constraint(m, x[e,l,c] == 0)

                # Symmetry rule: The cell and its mirror must have the same value
                elseif (l,c) < (lp,cp)
                    @constraint(m, x[e,l,c] - x[e,lp,cp] == 0)
                end
            end
        end
    end

    # =========================
    # Root constraints
    # =========================
    for e in 1:n_centers
        cx, cy = centers[e]
        for l in 1:n
            for c in 1:n
                # Check if the center point touches this cell
                is_root = abs(cx - (l-0.5)) <= 0.51 && abs(cy - (c-0.5)) <= 0.51

                if is_root
                    # Root cells are permanently assigned to their galaxy
                    @constraint(m, x[e,l,c] == 1)
                else
                    # Local connectivity constraint: 
                    # If a non-root cell belongs to a galaxy, it must have at least one neighbor from the same galaxy
                    @constraint(m,
                        x[e,l,c] <=
                        sum(
                            x[e,nl,nc]
                            for (nl,nc) in [
                                (l+1,c), (l-1,c), (l,c+1), (l,c-1)
                            ]
                            if 1 <= nl <= n && 1 <= nc <= n
                        )
                    )
                end
            end
        end
    end

    @objective(m, Min, 0)

    # =========================
    # BFS utility for the Callback
    # =========================
    # Finds all cells connected to the start_cell
    function get_component(cells::Set{Tuple{Int,Int}}, start_cell::Tuple{Int,Int})
        visited = Set{Tuple{Int,Int}}()
        queue = [start_cell]
        push!(visited, start_cell)

        dirs = [(1,0), (-1,0), (0,1), (0,-1)]

        while !isempty(queue)
            (l,c) = popfirst!(queue)
            for (dl,dc) in dirs
                nl = l + dl
                nc = c + dc
                if (nl,nc) in cells && !((nl,nc) in visited)
                    push!(visited, (nl,nc))
                    push!(queue, (nl,nc))
                end
            end
        end
        return visited
    end

    # =========================
    # Callback: Lazy Connectivity Cuts
    # =========================
    function callback_connectivity(cb_data::CPLEX.CallbackContext, context_id::Clong)
        
        # Only trigger on integer solutions
        if !isIntegerPoint(cb_data, context_id)
            return
        end

        CPLEX.load_callback_variable_primal(cb_data, context_id)
        x_val = callback_value.(cb_data, x)

        # Check each galaxy for isolated islands
        for e in 1:n_centers
            galaxy_cells = Set{Tuple{Int,Int}}()

            for l in 1:n
                for c in 1:n
                    if x_val[e,l,c] > 0.5
                        push!(galaxy_cells, (l,c))
                    end
                end
            end

            if isempty(galaxy_cells)
                continue
            end

            # Find the root cell to start the BFS
            cx, cy = centers[e]
            start_cell = (ceil(Int, cx), ceil(Int, cy))

            # Find the component physically connected to the root
            connected_component = get_component(galaxy_cells, start_cell)

            # If there are cells that cannot reach the root (an isolated island exists)
            if length(connected_component) < length(galaxy_cells)
                
                disconnected_part = setdiff(galaxy_cells, connected_component)
                Wset = Set(disconnected_part)

                # 1. Find the outer frontier of the isolated island
                frontier = Set{Tuple{Int,Int}}()
                for (l,c) in disconnected_part
                    for (nl,nc) in [(l+1,c), (l-1,c), (l,c+1), (l,c-1)]
                        if 1 <= nl <= n && 1 <= nc <= n
                            if !((nl,nc) in Wset)
                                push!(frontier, (nl,nc))
                            end
                        end
                    end
                end

                # 2. Create the connectivity cut
                if !isempty(frontier)
                    expr = AffExpr()
                    for (l,c) in frontier
                        add_to_expression!(expr, x[e,l,c])
                    end

                    # For every cell in the isolated island, force the frontier to open 
                    # ONLY IF the solver decides to keep this cell in the galaxy.
                    for (w_l, w_c) in Wset
                        cstr = @build_constraint(expr >= x[e, w_l, w_c])
                        MOI.submit(m, MOI.LazyConstraint(cb_data), cstr)
                    end
                end
            end
        end
    end

    # =========================
    # Register callback & Solve
    # =========================
    # Restrict to 1 thread to avoid callback race conditions in CPLEX
    MOI.set(m, MOI.NumberOfThreads(), 1)
    MOI.set(m, CPLEX.CallbackFunction(), callback_connectivity)

    start = time()
    optimize!(m)
    resTime = time() - start

    status = JuMP.primal_status(m) == MOI.FEASIBLE_POINT

    # =========================
    # Build solution
    # =========================
    solution_matrix = zeros(Int64, n, n)
    if status
        for l in 1:n
            for c in 1:n
                for e in 1:n_centers
                    if value(x[e,l,c]) > 0.5
                        solution_matrix[l,c] = e
                    end
                end
            end
        end
    end

    return status, resTime, solution_matrix
end

"""
Solve all the instances contained in dataFolder through CPLEX with callbacks
The results are written in resFolder
"""
function solveDataSetWithCallback()

    dataFolder = "./galaxy/data/"
    resFolder = "./galaxy/res/"
    
    # Specific folder for CPLEX with callback
    cplexCallbackFolder = resFolder * "cplex_callback/"

    # Create folders if they do not exist
    if !isdir(resFolder)
        mkdir(resFolder)
    end
    if !isdir(cplexCallbackFolder)
        mkdir(cplexCallbackFolder)
    end
            
    # For each .txt file in the data folder
    for file in filter(x->occursin(".txt", x), readdir(dataFolder))  
        
        println("-- Resolution of ", file)
        
        # Read the Galaxy instance data
        n, centers = readInputFile(dataFolder * file)
        outputFile = cplexCallbackFolder * file

        # Solve only if it hasn't been solved yet
        if !isfile(outputFile)
            
            fout = open(outputFile, "w")  
            
            # Run CPLEX with callback
            isOptimal, resolutionTime, solution_matrix = cplexSolveWithCallback(n, centers)

            # Save the results
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
            println("cplex callback optimal: ", isOptimal)
            println("cplex callback time: " * string(round(resolutionTime, sigdigits=2)) * "s\n")
        else
            println("Already solved! Skipping...")
        end         
    end 
end

"""
Determine if the callback was called due to finding an integer solution
"""
function isIntegerPoint(cb_data::CPLEX.CallbackContext, context_id::Clong)
    if context_id != CPX_CALLBACKCONTEXT_CANDIDATE
        return false
    end

    ispoint_p = Ref{Cint}()
    ret = CPXcallbackcandidateispoint(cb_data, ispoint_p)

    if ret != 0 || ispoint_p[] == 0
        return false
    else
        return true
    end
end