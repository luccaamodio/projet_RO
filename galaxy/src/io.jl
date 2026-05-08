# This file contains functions related to reading, writing and displaying a grid and experimental results

using JuMP
using Plots
import GR

"""
Save a grid in a text file

Argument
- n: size of the board
- centers: array with the centers
- outputFile: path of the output file
"""
function saveInstance(n::Int64, centers::Vector{Tuple{Float64, Float64}}, outputFile::String)

    # Open the output file
    writer = open(outputFile, "w")
    
    # First line is the size of the grid
    println(writer, n)
    
    # Next lines are the centers (cx, cy)
    for (cx, cy) in centers
        println(writer, cx, ",", cy)
    end
    
    close(writer)
end

"""
Read an instance from an input file

- Argument:
inputFile: path of the input file
"""
function readInputFile(inputFile::String)

    # Open the input file
    datafile = open(inputFile)

    data = readlines(datafile)
    close(datafile)

    # First line contains n
    n = parse(Int64, data[1])
    
    # List of the coordenates (x, y) of the centers
    centers = Vector{Tuple{Float64, Float64}}()

    # For each line of the input file (unless the first)
    for line in data[2:end]
        
        coords = split(line, ",")
        
        cx = parse(Float64, coords[1])
        cy = parse(Float64, coords[2])
        
        push!(centers, (cx, cy))
    end

    return n, centers
end

"""
Display a Galaxy instance from a file

Format:
- n: size o the grid
- centers: List of the coordenates (x, y) of the centers
"""
function displayGalaxy(n::Int64, centers::Vector{Tuple{Float64, Float64}})

    H = 2*n + 1
    W = 2*n + 1

    canvas = fill(' ', H, W)

    # intersections
    for l in 1:2:H
        for c in 1:2:W
            canvas[l,c] = '+'
        end
    end

    # horizontal edges
    for l in 1:2:H
        for c in 2:2:W-1
            canvas[l,c] = '-'
        end
    end

    # vertical edges
    for l in 2:2:H-1
        for c in 1:2:W
            canvas[l,c] = '|'
        end
    end

    # Draw centers
    for (l0, c0) in centers

        l = Int(round(2*l0+1))
        c = Int(round(2*c0+1))

        canvas[l,c] = '*'
    end

    # Print
    for l in 1:H
        for c in 1:W
            print(canvas[l,c], ' ')
        end
        println()
    end
end

"""
Display cplex solution

Argument
- x: 3-dimensional variables array such that x[i, j, k] = 1 if cell (i, j) has value k
"""
function displaySolution(solution_matrix::Matrix{Int64})
    n = size(solution_matrix, 1)

    # Horizontal separator between rows l and l+1 (or border if l=0 or l=n)
    function hline(l::Int)
        row = "+"
        for c in 1:n
            above = l == 0 ? -1 : solution_matrix[l, c]
            below = l == n ? -1 : solution_matrix[l+1, c]
            row *= (above != below ? "----+" : "    +")
        end
        return row
    end

    println(hline(0))
    for l in 1:n
        row = "|"
        for c in 1:n
            row *= rpad(solution_matrix[l, c], 3) * " "
            right = c == n ? -1 : solution_matrix[l, c+1]
            row *= (solution_matrix[l, c] != right ? "|" : " ")
        end
        println(row)
        println(hline(l))
    end
end

"""
Create a pdf file which contains a performance diagram associated to the results of the ../res folder
Display one curve for each subfolder of the ../res folder.

Arguments
- outputFile: path of the output file

Prerequisites:
- Each subfolder must contain text files
- Each text file correspond to the resolution of one instance
- Each text file contains a variable "solveTime" and a variable "isOptimal"
"""
function performanceDiagram(outputFile::String)

    resultFolder = "./galaxy/res/"
    
    # Maximal number of files in a subfolder
    maxSize = 0

    # Number of subfolders
    subfolderCount = 0

    folderName = Array{String, 1}()

    # For each file in the result folder
    for file in readdir(resultFolder)

        path = resultFolder * file
        
        # If it is a subfolder
        if isdir(path)
            
            folderName = vcat(folderName, file)
             
            subfolderCount += 1
            folderSize = size(readdir(path), 1)

            if maxSize < folderSize
                maxSize = folderSize
            end
        end
    end

    # Array that will contain the resolution times (one line for each subfolder)
    results = Array{Float64}(undef, subfolderCount, maxSize)

    for i in 1:subfolderCount
        for j in 1:maxSize
            results[i, j] = Inf
        end
    end

    folderCount = 0
    maxSolveTime = 0

    # For each subfolder
    for file in readdir(resultFolder)
            
        path = resultFolder * file
        
        if isdir(path)

            folderCount += 1
            fileCount = 0

            # For each text file in the subfolder
            for resultFile in filter(x->occursin(".txt", x), readdir(path))

                fileCount += 1

                fullpath = joinpath(path, resultFile)
                solveTime, isOptimal = readResult(fullpath)

                if isOptimal
                    results[folderCount, fileCount] = solveTime

                    if solveTime > maxSolveTime
                        maxSolveTime = solveTime
                    end 
                end 
            end 
        end
    end 

    # Sort each row increasingly
    results = sort(results, dims=2)

    println("Max solve time: ", maxSolveTime)

    # For each line to plot
    for dim in 1: size(results, 1)

        x = Array{Float64, 1}()
        y = Array{Float64, 1}()

        # x coordinate of the previous inflexion point
        previousX = 0
        previousY = 0

        append!(x, previousX)
        append!(y, previousY)
            
        # Current position in the line
        currentId = 1

        # While the end of the line is not reached 
        while currentId != size(results, 2) && results[dim, currentId] != Inf

            # Number of elements which have the value previousX
            identicalValues = 1

             # While the value is the same
            while results[dim, currentId] == previousX && currentId <= size(results, 2)
                currentId += 1
                identicalValues += 1
            end

            # Add the proper points
            append!(x, previousX)
            append!(y, currentId - 1)

            if results[dim, currentId] != Inf
                append!(x, results[dim, currentId])
                append!(y, currentId - 1)
            end
            
            previousX = results[dim, currentId]
            previousY = currentId - 1
            
        end

        append!(x, maxSolveTime)
        append!(y, currentId - 1)

        p = plot(
            xlabel = "Time (s)",
            ylabel = "Solved instances",
            legend = :bottomright
        )

        for dim in 1:size(results, 1)

            x = Float64[]
            y = Float64[]

            previousX = 0
            currentId = 1

            push!(x, 0.0)
            push!(y, 0.0)

            while currentId <= size(results, 2) && results[dim, currentId] != Inf

                while currentId <= size(results, 2) && results[dim, currentId] == previousX
                    currentId += 1
                end

                push!(x, previousX)
                push!(y, currentId - 1)

                if currentId <= size(results, 2) && results[dim, currentId] != Inf
                    push!(x, results[dim, currentId])
                    push!(y, currentId - 1)
                end

                previousX = currentId <= size(results, 2) ? results[dim, currentId] : previousX
            end

            push!(x, maxSolveTime)
            push!(y, currentId - 1)

            plot!(p, x, y, label = folderName[dim], linewidth=3)
        end

        savefig(p, outputFile)
    end
end

function readResult(path::String)
    solveTime = NaN
    isOptimal = false

    for line in eachline(path)
        if occursin("solveTime", line)
            solveTime = parse(Float64, split(line, "=")[2])
        elseif occursin("isOptimal", line)
            isOptimal = occursin("true", lowercase(line))
        end
    end

    return solveTime, isOptimal
end

"""
Create a latex file which contains an array with the results of the ../res folder.
Each subfolder of the ../res folder contains the results of a resolution method.

Arguments
- outputFile: path of the output file

Prerequisites:
- Each subfolder must contain text files
- Each text file correspond to the resolution of one instance
- Each text file contains a variable "solveTime" and a variable "isOptimal"
"""
function resultsArray(outputFile::String)

    resultFolder = "./galaxy/res/"
    dataFolder = "./galaxy/data/"

    # Maximal number of files in a subfolder
    maxSize = 0

    # Number of subfolders
    subfolderCount = 0

    # Open the latex output file
    fout = open(outputFile, "w")

    # Print the latex file output
    println(fout, raw"""\documentclass{article}

\usepackage[french]{babel}
\usepackage [utf8] {inputenc} % utf-8 / latin1 
\usepackage{multicol}

\setlength{\hoffset}{-18pt}
\setlength{\oddsidemargin}{0pt} % Marge gauche sur pages impaires
\setlength{\evensidemargin}{9pt} % Marge gauche sur pages paires
\setlength{\marginparwidth}{54pt} % Largeur de note dans la marge
\setlength{\textwidth}{481pt} % Largeur de la zone de texte (17cm)
\setlength{\voffset}{-18pt} % Bon pour DOS
\setlength{\marginparsep}{7pt} % Séparation de la marge
\setlength{\topmargin}{0pt} % Pas de marge en haut
\setlength{\headheight}{13pt} % Haut de page
\setlength{\headsep}{10pt} % Entre le haut de page et le texte
\setlength{\footskip}{27pt} % Bas de page + séparation
\setlength{\textheight}{668pt} % Hauteur de la zone de texte (25cm)

\begin{document}""")

    header = raw"""
\begin{center}
\renewcommand{\arraystretch}{1.4}
\begin{tabular}{l"""

# Name of the subfolder of the result folder (i.e, the resolution methods used)
    folderName = Array{String, 1}()
    
    # List of all the instances solved by at least one resolution method
    solvedInstances = Array{String, 1}()

    # For each file in the result folder
    for file in readdir(resultFolder)

        path = resultFolder * file

        # If it is a subfolder
        if isdir(path)

            # Add its name to the folder list
            folderName = vcat(folderName, file)
             
            subfolderCount += 1
            folderSize = size(readdir(path), 1)

            # Add all its files in the solvedInstances array
            for file2 in filter(x->occursin(".txt", x), readdir(path))
                solvedInstances = vcat(solvedInstances, file2)
            end 

            if maxSize < folderSize
                maxSize = folderSize
            end
        end
    end

    # Only keep one string for each instance solved
    unique(solvedInstances)

    # For each resolution method, add two columns in the array
    for folder in folderName
        header *= "rr"
    end

    header *= "}\n\t\\hline\n"

    # Create the header line which contains the methods name
    for folder in folderName
        header *= " & \\multicolumn{2}{c}{\\textbf{" * folder * "}}"
    end

    header *= "\\\\\n\\textbf{Instance} "

    # Create the second header line with the content of the result columns
    for folder in folderName
        header *= " & \\textbf{Temps (s)} & \\textbf{Optimal ?} "
    end

    header *= "\\\\\\hline\n"

    footer = raw"""\hline\end{tabular}
\end{center}

"""
    println(fout, header)

    # On each page an array will contain at most maxInstancePerPage lines with results
    maxInstancePerPage = 30
    id = 1

    # For each solved files
    for solvedInstance in solvedInstances

        # If we do not start a new array on a new page
        if rem(id, maxInstancePerPage) == 0
            println(fout, footer, "\\newpage")
            println(fout, header)
        end

        # Replace the potential underscores '_' in file names
        print(fout, replace(solvedInstance, "_" => "\\_"))

        # For each resolution method
        for method in folderName

            path = resultFolder * method * "/" * solvedInstance

            # If the instance has been solved by this method
            if isfile(path)

                solveTime, isOptimal = readResult(path)

                println(fout, " & ", round(solveTime, digits=2), " & ")

                if isOptimal
                    println(fout, "\$\\times\$")
                end

            # If the instance has not been solved by this method
            else
                println(fout, " & - & - ")
            end
        end

        println(fout, "\\\\")

        id += 1
    end

    # Print the end of the latex file
    println(fout, footer)

    println(fout, "\\end{document}")

    close(fout)

end