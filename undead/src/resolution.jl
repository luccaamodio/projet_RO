# This file contains methods to solve an instance (heuristically or with CPLEX)
using CPLEX

include("generation.jl")

TOL = 0.00001

"""
Returns a list with the positions viewed directly and another with the ones viewed through a mirror
"""
function positionsViewed(t::Matrix{Char}, n::Int64, x::Int64, y::Int64, dx::Int64, dy::Int64)
    reflected = false
    direct_positions = Vector{Tuple{Int64, Int64}}()
    reflected_positions = Vector{Tuple{Int64, Int64}}()
    while 1 <= x <= n && 1 <= y <= n
        if t[x, y] == 'b'
            reflected = true
            temp = dx
            dx = dy
            dy = temp
        elseif t[x, y] == 'c'
            reflected = true
            temp = dx
            dx = -dy
            dy = -temp
            end
        else
            if reflected
                push!(reflected_positions, (x, y))
            else
                push!(direct_positions, (x, y))
            end
        end
        x += dx
        y += dy
    end

    return direct_positions, reflected_positions

end

"""
Solve an instance with CPLEX
"""
function cplexSolve(t::Matrix{Char}, views::Matrix{Int64})

    # Create the model
    m = Model(with_optimizer(CPLEX.Optimizer))

    n = size(t, 2)

    # x[i, j, 1] = 1 if cell (i, j) is a ghost
    # x[i, j, 2] = 1 if cell (i, j) is a vampire
    # x[i, j, 3] = 1 if cell (i, j) is a monster
    @variable(m, x[1:n, 1:n, 1:3], Bin)

    # les positions des miroir ne peuvent avoir aucune valeur
    # toutes les positions non miroir ne peuvent avoir qu'une valeur
    for l in 1:n
        for c in 1:n
            if t[l, c] == 'b' || t[l, c] == 'c'
                @constraint(m, [k in 1:3], x[l,c,k] == 0)
            else
                @constraint(m, sum(x[l, c, i] for i in 1:3) == 1)
            end
        end
    end

    # The number of views has to be compatible with the solution
    # To verify that, we are going to create a Matrix of coeficients representing
    # the visibility of a cell. It has an element representing "directly visible" and
    # another representing "visible by mirror"

    # C[i, j, 1] is direct visibility of the (i, j) cell
    # C[i, j, 2] is visibility by mirror of the (i, j) cell

    # These visibilities can be greater than 1 if the cell is crossed twice

    # Todo: Optimize the code using fewer elements of C. Maybe get only those from
    # the visibility lists

    # For the Left side counts
    for i in 1:n
        C = Matrix{Int64}(zeros(n, n, 2))
        dir_pos, ref_pos = positionsViewed(t, n, 1, i, 1, 0)
        for dp in dir_pos
            (l,c) = dp
            C[l,c,1] += 1
        end
        for rp in ref_pos
            (l,c) = rp
            C[l,c,2] += 1
        end

        @constraint(m,
            sum((x[l,c,1] + x[l,c,3]) * C[l,c,2] +
                (x[l,c,2] + x[l,c,3]) * C[l,c,1]
                for l in 1:n, c in 1:n)
            == views[1, i]
        )
    end

    # For the Bottom side counts
    for i in 1:n
        C = Matrix{Int64}(zeros(n, n, 2))
        dir_pos, ref_pos = positionsViewed(t, n, i, 1, 0, 1)
        for dp in dir_pos
            (l,c) = dp
            C[l,c,1] += 1
        end
        for rp in ref_pos
            (l,c) = rp
            C[l,c,2] += 1
        end
        @constraint(m,
            sum((x[l,c,1] + x[l,c,3]) * C[l,c,2] +
                (x[l,c,2] + x[l,c,3]) * C[l,c,1]
                for l in 1:n, c in 1:n)
            == views[2, i]
        )
    end
        
    # For the Right side counts
    for i in 1:n
        C = Matrix{Int64}(zeros(n, n, 2))
        dir_pos, ref_pos = positionsViewed(t, n, n, i, -1, 0)
        for dp in dir_pos
            (l,c) = dp
            C[l,c,1] += 1
        end
        for rp in ref_pos
            (l,c) = rp
            C[l,c,2] += 1
        end

        @constraint(m,
            sum((x[l,c,1] + x[l,c,3]) * C[l,c,2] +
                (x[l,c,2] + x[l,c,3]) * C[l,c,1]
                for l in 1:n, c in 1:n)
            == views[3, i]
        )
    end
        
    # For the Up side counts
    for i in 1:n
        C = Matrix{Int64}(zeros(n, n, 2))
        dir_pos, ref_pos = positionsViewed(t, n, i, n, 0, -1)
        for dp in dir_pos
            (l,c) = dp
            C[l,c,1] += 1
        end
        for rp in ref_pos
            (l,c) = rp
            C[l,c,2] += 1
        end
        
        @constraint(m,
            sum((x[l,c,1] + x[l,c,3]) * C[l,c,2] +
                (x[l,c,2] + x[l,c,3]) * C[l,c,1]
                for l in 1:n, c in 1:n)
            == views[4, i]
        )
    end

    # TODO
    println("In file resolution.jl, in method cplexSolve(), TODO: fix input and output, define the model")

    # Start a chronometer
    start = time()

    # Solve the model
    optimize!(m)

    # Return:
    # 1 - true if an optimum is found
    # 2 - the resolution time
    return JuMP.primal_status(m) == JuMP.MathOptInterface.FEASIBLE_POINT, time() - start
    
end

"""
Heuristically solve an instance
"""
function heuristicSolve()

    # TODO
    println("In file resolution.jl, in method heuristicSolve(), TODO: fix input and output, define the model")
    
end 

"""
Solve all the instances contained in "../data" through CPLEX and heuristics

The results are written in "../res/cplex" and "../res/heuristic"

Remark: If an instance has previously been solved (either by cplex or the heuristic) it will not be solved again
"""
function solveDataSet()

    dataFolder = "../data/"
    resFolder = "../res/"

    # Array which contains the name of the resolution methods
    resolutionMethod = ["cplex"]
    #resolutionMethod = ["cplex", "heuristique"]

    # Array which contains the result folder of each resolution method
    resolutionFolder = resFolder .* resolutionMethod

    # Create each result folder if it does not exist
    for folder in resolutionFolder
        if !isdir(folder)
            mkdir(folder)
        end
    end
            
    global isOptimal = false
    global solveTime = -1

    # For each instance
    # (for each file in folder dataFolder which ends by ".txt")
    for file in filter(x->occursin(".txt", x), readdir(dataFolder))  
        
        println("-- Resolution of ", file)
        readInputFile(dataFolder * file)

        # TODO
        println("In file resolution.jl, in method solveDataSet(), TODO: read value returned by readInputFile()")
        
        # For each resolution method
        for methodId in 1:size(resolutionMethod, 1)
            
            outputFile = resolutionFolder[methodId] * "/" * file

            # If the instance has not already been solved by this method
            if !isfile(outputFile)
                
                fout = open(outputFile, "w")  

                resolutionTime = -1
                isOptimal = false
                
                # If the method is cplex
                if resolutionMethod[methodId] == "cplex"
                    
                    # TODO 
                    println("In file resolution.jl, in method solveDataSet(), TODO: fix cplexSolve() arguments and returned values")
                    
                    # Solve it and get the results
                    isOptimal, resolutionTime = cplexSolve()
                    
                    # If a solution is found, write it
                    if isOptimal
                        # TODO
                        println("In file resolution.jl, in method solveDataSet(), TODO: write cplex solution in fout") 
                    end

                # If the method is one of the heuristics
                else
                    
                    isSolved = false

                    # Start a chronometer 
                    startingTime = time()
                    
                    # While the grid is not solved and less than 100 seconds are elapsed
                    while !isOptimal && resolutionTime < 100
                        
                        # TODO 
                        println("In file resolution.jl, in method solveDataSet(), TODO: fix heuristicSolve() arguments and returned values")
                        
                        # Solve it and get the results
                        isOptimal, resolutionTime = heuristicSolve()

                        # Stop the chronometer
                        resolutionTime = time() - startingTime
                        
                    end

                    # Write the solution (if any)
                    if isOptimal

                        # TODO
                        println("In file resolution.jl, in method solveDataSet(), TODO: write the heuristic solution in fout")
                        
                    end 
                end

                println(fout, "solveTime = ", resolutionTime) 
                println(fout, "isOptimal = ", isOptimal)
                
                # TODO
                println("In file resolution.jl, in method solveDataSet(), TODO: write the solution in fout") 
                close(fout)
            end


            # Display the results obtained with the method on the current instance
            include(outputFile)
            println(resolutionMethod[methodId], " optimal: ", isOptimal)
            println(resolutionMethod[methodId], " time: " * string(round(solveTime, sigdigits=2)) * "s\n")
        end         
    end 
end
