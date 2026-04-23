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

    t = []
    t = Matrix{Char}(undef, n, n)
    # Drafts the numbers of each character
    mirrors = rand(n^2/8:n^2/3)
    vampires = rand(0:n^2 - mirrors)
    monsters = rand(0:n^2 - mirrors - vampires)
    fantasmas = n^2 - mirrors - vampires - monsters

    positions = [(i,j) for i in 1:n for j in 1:n]
    shuffle!(positions)
    idx = 1

    for mirror in 1:mirrors
        (x, y) = pop!(positions)
        mirror_seed = rand() # direction draft
        if mirror_seed < 0.5
            t[x, y] = 'b'
        else
            t[x, y] = 'c'
        end
    end
    
    for fantasma in 1:fantasmas
        (x, y) = pop!(positions)
        t[x, y] = 'f'
    end
    for monster in 1:monsters
        (x, y) = pop!(positions)
        t[x, y] = 'm'
    end
    for vampire in 1:vampires
        (x, y) = pop!(positions)
        t[x, y] = 'v'
    end

    views = []
    views = Matrix{Int}(zeros(4, n))
    for i in 1:n
        # Right side count line
        views[1, i] = countCharacter(t, n, 1, i, 1, 0)
        # Bottom side count line
        views[2, i] = countCharacter(t, n, i, 1, 0, 1)
        # Left side count line
        views[3, i] = countCharacter(t, n, n, i, -1, 0)
        # Up side count line
        views[4, i] = countCharacter(t, n, i, n, 0, -1)
    end
    return t, views

end

function countCharacter(t::Matrix{Char}, n::Int64, x::Int64, y::Int64, dx::Int64, dy::Int64)
    reflected = false
    count = 0
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
        else
            if reflected
                if t[x, y] == 'f' || t[x, y] == 'z'
                    count += 1
                end
            else
                if t[x, y] == 'm' || t[x, y] == 'z'
                    count += 1
                end
            end
        end
        x += dx
        y += dy
    end

    return Char(count)

end

"""
Generate all the instances

Remark: a grid is generated only if the corresponding output file does not already exist
"""
function generateDataSet()

    # For each grid size considered
    #for size in [4, 9, 16, 25]
    for size in [16]

        # For each grid density considered
        #for density in 0.1:0.2:0
        for density in 0.1

            # Generate 10 instances
            for instance in 1:10

                fileName = "undead/data/instance_t" * string(size) * "_d" * string(density) * "_" * string(instance) * ".txt"
                
                if !isfile(fileName)
                    println("-- Generating file " * fileName)
                    t, views = generateInstance(size, density)
                    saveInstance(t, views, fileName)
                end 
            end
        end
    end
end


