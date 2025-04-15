function Main()
    local vector = {1, 2, 3, x = 1, y = 2, z = 3}

    for i = 1, #vector do
        print(i, vector[i])
    end
    print("Length: ", #vector)
end

Main ()