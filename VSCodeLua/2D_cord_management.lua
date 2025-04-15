-- 2D coordinate management system

-- multiply 2 matrices
function matrix_mult(a, b)
    local result = {}
    for i = 1, #a[1] do
        result[i] = {}
        for j = 1, #b do
            result[i][j] = result[i][j] + a[i][j]*b[j][i]
        end
    end
    return result
end


function matrix_vector_mult(matrix, vector)
    local result = {}
    for i = 1, #matrix do
        result[i] = 0
        for j = 1, #vector do
            result[i] = result[i] + matrix[i][j] * vector[j]
        end
    end
    return result
end


function matrix_inverse(m)
    -- Extract the translation vector
    local translation = {}
    for i = 1, #m-1 do
        translation[i] = -m[i][#m]
    end


    -- transpose the rotation part of the matrix
    local m_transpose = {}
    for i = 1, #m-1 do
        m_transpose[i] = {}
        for j = 1, #m-1 do
            m_transpose[i][j] = m[j][i]
        end
    end

    -- -- multiply the translation by the transpose
    -- local translation_transpose = matrix_vector_mult(m_transpose, translation)

    -- Compute the new translation vector
    local new_translation = {}
    for i = 1, #m - 1 do
        new_translation[i] = 0
        for j = 1, #m - 1 do
            new_translation[i] = new_translation[i] + m_transpose[i][j] * translation[j]
        end
    end

    -- append the translation to the transpose
    for i = 1, #m-1 do
        m_transpose[i][#m] = new_translation[i]
    end

    -- Add the homogeneous row
    m_transpose[#m] = {}
    for j = 1, #m do
        m_transpose[#m][j] = (j == #m) and 1 or 0
    end

    return m_transpose
end

function print_matrix(matrix)
    for i = 1, #matrix do
        for j = 1, #matrix[i] do
            io.write(matrix[i][j] .. "\t")
        end
        io.write("\n")
    end
end

function print_vector(vector)
    for i = 1, #vector do
        io.write(vector[i] .. "\t")
    end
    io.write("\n")
end

Point = {}
Point.__index = Point

function Point:new(coords, frame) -- coords has x, y, and 1 for homogenous coordinates
    local instance = {}
    setmetatable(instance, Point)
    instance.coords = coords
    instance.frame = frame
    return instance
end

-- converts a point's coordinates to a new frame
function Point:to(newFrame)
    -- local world_coords = matrix_vector_mult(matrix_inverse(self.frame), self.coords) 
    -- return matrix_vector_mult(named_frames[newFrame], world_coords)

    -- Get the world coordinates of the point
    local world_coords = matrix_vector_mult(matrix_inverse(self.frame.transform), self.coords)

    -- Transform to the new frame
    local new_coords = matrix_vector_mult(named_frames[newFrame].transform, world_coords)

    -- Return a new Point in the new frame
    return Point:new(new_coords, newFrame)
end


Directional = {}
Directional.__index = Directional

function Directional:new(coords, frame) -- coords has x, y and 0 for homogeneous coordinates
    local instance = {}
    setmetatable(instance, Directional)
    instance.coords = coords
    instance.frame = frame
    return instance
end


-- direconals are only effected by rotation
function Directional:to(newFrame)
    -- Extract the rotation part of the matrix (ignore translation)
    local rotation_matrix = {}
    for i = 1, 2 do
        rotation_matrix[i] = {named_frames[newFrame].transform[i][1], named_frames[newFrame].transform[i][2]}
    end

    -- Transform the direction using only rotation
    local new_coords = matrix_vector_mult(rotation_matrix, self.coords)

    -- Return a new Directional in the new frame
    return Directional:new(new_coords, newFrame)
end


named_frames = {}

frame = {}
frame.__index = frame

function frame:new(name, origin, X, Y)
    local instance = {}
    setmetatable(instance, frame)
    named_frames[name] = instance
    instance.transform = {}
    for i = 1, #X do
        instance.transform[i] = {X[i], Y[i], origin[i]}
    end
    instance.transform[#X + 1] = {0, 0, 1}
    instance.origin = origin
    return instance
end


-- Frame Definitions
local world = frame:new("world", {0, 0}, {1, 0}, {0, 1}) -- Identity frame
local frameA = frame:new("frameA", {2, 3}, {0, 1}, {-1, 0}) -- 90° rotation + translation
local frameB = frame:new("frameB", {-1, -2}, {0.5, 0}, {0, 0.5}) -- Scaling + translation

-- Point and Directional Definitions
local pointA = Point:new({1, 1}, frameA)
local directionA = Directional:new({1, 0}, frameA)

print_matrix(frameA.transform)
print_matrix(matrix_inverse(frameA.transform))

local point_in_world = pointA:to("world")
print("Point in world frame:", point_in_world.coords[1], point_in_world.coords[2])
-- Expected Output: A point transformed using frameA's inverse transformation