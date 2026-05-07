# This file contains methods to generate a data set of instances (i.e., sudoku grids)
using Random
using Pkg
Pkg.precompile()
include("io.jl")

"""
Generate an n*n grid with a given density

Argument
- n: size of the grid
- density: percentage in [0, 1] of initial values in the grid
"""
function generateInstance(n::Int64, density::Float64)

    grid = zeros(Int64, n, n)
    centers = Vector{Tuple{Float64, Float64}}()
    
    # Calculate how many starting seeds to plant (e.g., 10% of the grid size)
    num_seeds = max(1, round(Int, density*n^2))
    
    # ==========================================
    # 1. PLANT SEEDS PHASE
    # ==========================================
    for id in 1:num_seeds
        
        # Find all empty cells
        empty_cells = [(l, c) for l in 1:n for c in 1:n if grid[l, c] == 0]
        if isempty(empty_cells)
            break
        end
        
        # Pick a random empty cell
        l, c = rand(empty_cells)
        
        # Randomly choose the shape of the seed (1x1, 1x2, 2x1, or 2x2)
        seed_type = rand(1:4)
        
        if seed_type == 4 && l < n && c < n && grid[l, c+1] == 0 && grid[l+1, c] == 0 && grid[l+1, c+1] == 0
            # 2x2 block
            grid[l, c] = id; # UpLeft
            grid[l, c+1] = id; # UpRight
            grid[l+1, c] = id; # DownLeft
            grid[l+1, c+1] = id; # DownRight
            push!(centers, (l, c))
            
        elseif seed_type == 3 && l < n && grid[l+1, c] == 0
            # 2x1 vertical block
            grid[l, c] = id; # Up
            grid[l+1, c] = id; # Down
            push!(centers, (Float64(l), c-0.5))
            
        elseif seed_type == 2 && c < n && grid[l, c+1] == 0
            # 1x2 horizontal block
            grid[l, c] = id; # Left
            grid[l, c+1] = id; # Right
            push!(centers, (l-0.5, Float64(c)))
            
        else
            # 1x1 block
            grid[l, c] = id
            push!(centers, (l-0.5, c-0.5))
        end
    end
    
    # ==========================================
    # 2. GROW PHASE (Expand symmetrically)
    # ==========================================
    changed = true
    while changed
        changed = false
        
        # Try to grow each galaxy one by one
        for id in 1:length(centers)
            cx, cy = centers[id]
            
            # Find empty cells that are touching this galaxy
            adjacent = []
            for l in 1:n
                for c in 1:n
                    if grid[l, c] == 0
                        # Check neighbors (up, down, left, right)
                        is_touching = (l > 1 && grid[l-1, c] == id) || 
                                      (l < n && grid[l+1, c] == id) || 
                                      (c > 1 && grid[l, c-1] == id) || 
                                      (c < n && grid[l, c+1] == id)
                        
                        if is_touching
                            push!(adjacent, (l, c))
                        end
                    end
                end
            end
            
            # Shuffle to grow in random directions
            shuffle!(adjacent)
            
            for (l, c) in adjacent
                # Skip if it was filled by another galaxy in the same round
                if grid[l, c] != 0
                    continue
                end
                
                # Calculate the exact mirror/symmetric position
                lp = round(Int, 2.0 * cx - l + 1)
                cp = round(Int, 2.0 * cy - c + 1)
                
                # If the mirror position is inside the grid and is also empty
                if lp >= 1 && lp <= n && cp >= 1 && cp <= n
                    if grid[lp, cp] == 0
                        
                        # Add both the cell and its mirror to the galaxy
                        grid[l, c] = id
                        grid[lp, cp] = id
                        changed = true
                        
                        break # Move to the next galaxy to keep them growing balanced
                    end
                end
            end
        end
    end
    
    # ==========================================
    # 3. FILL PHASE (Leftovers become 1x1)
    # ==========================================
    id = length(centers)
    for l in 1:n
        for c in 1:n
            if grid[l, c] == 0
                id += 1
                grid[l, c] = id
                push!(centers, (l-0.5, c-0.5))
            end
        end
    end
    
    return centers
end

"""
Generate all the instances

Remark: a grid is generated only if the corresponding output file does not already exist
"""
function generateDataSet()

    # For each grid size considered
    for size in [4, 9, 16, 25]

        # For each grid density considered
        for density in 0.1:0.2:0.3

            # Generate 10 instances
            # for instance in 1:10
            for instance in 1:2

                fileName = "galaxy/data/instance_t" * string(size) * "_d" * string(density) * "_" * string(instance) * ".txt"
                
                if !isfile(fileName)
                    println("-- Generating file " * fileName)
                    centers = generateInstance(size, density)
                    saveInstance(size, centers, fileName)
                end 
            end
        end
    end
end