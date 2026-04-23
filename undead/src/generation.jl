# This file contains methods to generate a data set of instances (i.e., sudoku grids)
using Random
include("io.jl")

"""
Generate an n*n grid with a given density

Argument
- n: size of the grid
- density: percentage in [0, 1] of initial values in the grid
"""
function generateInstance(n::Int64, density::Float64)


    t = []
    t = Matrix{Int}(zeros(n, n))
    # O código correto deve ser:
    mirrors = rand(n^2/8:n^2/3) # Sorteia UM número entre 2n e 3n
    vampires = rand(0:n^2 - mirrors)
    monsters = rand(0:n^2 - mirrors - vampires)
    fantasmas = rand(0:n^2 - mirrors - vampires - monsters)
    
    for mirror in mirrors
        mirror_seed = rand(0,1) #direction draft
        pos_x = rand(0,n)
        pos_y = rand(0,n)
        while t[pos_x][pos_y]!=0
            pos_x = rand(0,n)
            pos_y = rand(0,n)
        end
        if mirror_seed <0.5

            t[pos_x][pos_y] = 'b'
        else
            t[pos_x][pos_y] = 'c'
        end
    end
    
    for fantasma in fantasmas
        pos_x = rand(0,n)
        pos_y = rand(0,n)
        while t[pos_x][pos_y]!=0
            pos_x = rand(0,n)
            pos_y = rand(0,n)
        end
        t[pos_x][pos_y] = 'f'
    end
    for monster in monsters
        pos_x = rand(0,n)
        pos_y = rand(0,n)
        while t[pos_x][pos_y]!=0
            pos_x = rand(0,n)
            pos_y = rand(0,n)
        end
        t[pos_x][pos_y] = 'm'
    end
    for vampire in vampires
        pos_x = rand(0,n)
        pos_y = rand(0,n)
        while t[pos_x][pos_y]!=0
            pos_x = rand(0,n)
            pos_y = rand(0,n)
        end
        t[pos_x][pos_y] = 'v'
    end
    println("In file generation.jl, in method generateInstance(), TODO: generate an instance")
    
end 

"""
Generate all the instances

Remark: a grid is generated only if the corresponding output file does not already exist
"""
function generateDataSet()

    # TODO
    println("In file generation.jl, in method generateDataSet(), TODO: generate an instance")
    
end



