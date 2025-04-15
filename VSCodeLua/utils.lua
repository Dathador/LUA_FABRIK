function MatrixMult(a, b)
    local result = {}
    for i = 1, #a do
        result[i] = {}
        for j = 1, #b[1] do
            result[i][j] = 0
            for k = 1, #b do
                result[i][j] = result[i][j] + a[i][k] * b[k][j]
            end
        end
    end
    return result
end

function Distance(p1, p2)
    local squared_distance = 0
    for i = 1, #p1 do
        squared_distance = squared_distance + (p1[i] - p2[i])^2
    end
    return math.sqrt(squared_distance)
end

function Subtract(p1, p2)
    local result = {}
    for i = 1, #p1 do
        result[i] = p1[i] - p2[i]
    end
    return result
end

function Add(p1, p2)
    local result = {}
    for i = 1, #p1 do
        result[i] = p1[i] + p2[i]
    end
    return result
end

function Length(p)
    local squared_length = 0
    for i = 1, #p do
        squared_length = squared_length + p[i]^2
    end
    return math.sqrt(squared_length)
end

function Normalize(p)
    local length = Length(p)
    if length == 0 then
        return {0, 0, 0}
    end
    local result = {}
    for i = 1, #p do
        result[i] = p[i] / length
    end
    return result
end

function Multiply(p, scalar)
    local result = {}
    for i = 1, #p do
        result[i] = p[i] * scalar
    end
    return result
end

function Dot(p1, p2)
    local result = 0
    for i = 1, #p1 do
        result = result + p1[i] * p2[i]
    end
    return result
end

function Cross(p1, p2)
    if #p1 == 2 and #p2 == 2 then
        return p1[1] * p2[2] - p1[2] * p2[1]
    elseif #p1 == 3 and #p2 == 3 then
        return {
            p1[2] * p2[3] - p1[3] * p2[2],
            p1[3] * p2[1] - p1[1] * p2[3],
            p1[1] * p2[2] - p1[2] * p2[1]
        }
    else
        error("Cross product is only defined for 2D or 3D vectors")
    end
end

function Angle(p1, p2)
    local dot_product = Dot(p1, p2)
    local length_product = Length(p1) * Length(p2)
    if length_product == 0 then
        return 0
    end
    return math.acos(dot_product / length_product)
end

function RotationMatrix(angle, axis)
    local cos_angle = math.cos(angle)
    local sin_angle = math.sin(angle)
    local one_minus_cos = 1 - cos_angle

    return {
        {cos_angle + axis[1] * axis[1] * one_minus_cos                      ,             axis[1] * axis[2] * one_minus_cos - axis[3] * sin_angle,             axis[1] * axis[3] * one_minus_cos + axis[2] * sin_angle},
        {            axis[2] * axis[1] * one_minus_cos + axis[3] * sin_angle, cos_angle + axis[2] * axis[2] * one_minus_cos                      ,             axis[2] * axis[3] * one_minus_cos - axis[1] * sin_angle},
        {            axis[3] * axis[1] * one_minus_cos - axis[2] * sin_angle,             axis[3] * axis[2] * one_minus_cos + axis[1] * sin_angle, cos_angle + axis[3] * axis[3] * one_minus_cos}
    }
end

function RotateVector(vector, angle, axis)
    local rotation_matrix = RotationMatrix(angle, axis)
    return {
        rotation_matrix[1][1] * vector[1] + rotation_matrix[1][2] * vector[2] + rotation_matrix[1][3] * vector[3],
        rotation_matrix[2][1] * vector[1] + rotation_matrix[2][2] * vector[2] + rotation_matrix[2][3] * vector[3],
        rotation_matrix[3][1] * vector[1] + rotation_matrix[3][2] * vector[2] + rotation_matrix[3][3] * vector[3]
    }
end

function RotateAround(vector, angle, axis, center)
    local rotated = RotateVector(Subtract(vector, center), angle, axis)
    return Add(rotated, center)
end

function RotatePointAroundAxis(p, angle, axis, center)
    -- Rotate point p around axis through center by angle
    local v = Subtract(p, center)
    local v_rot = RotateVector(v, angle, axis)
    return Add(center, v_rot)
end

function Clamp(value, min, max)
    return math.max(min, math.min(value, max))
end

-- function to produce a smoth transition between two values
-- movement_start and movement_end change the start and stop of the curve
-- a and b change the shape of the curve
function Easing(interpolation_percent, a, b)
    --[[
    a=1,b=1 linear
    a>1,b=1 quadratic
    a<1,b>1 exponential  ( a=(10-b)/10 rather pretty )
    a>1,b<1 logarithmic  ( a=1.3,b=0.5 | a=1.7,b=0.3 | a=2,b=0.2 | a=3.2,b=0.1) ( a=( 1+(1-b)/2 )*10/9 rather pretty )
    ]]
    local movementProgress = Clamp(interpolation_percent,0,1)
    local interpolatedValue
    if movementProgress <= 0.5 then
        interpolatedValue = 0.5*(2*movementProgress)^a
    else
        interpolatedValue = 1-0.5*(-2*movementProgress+2)^a
    end
    return interpolatedValue^b
end

function EasePosition(startPos, endPos, timer, a, b)
    local v = Subtract(endPos, startPos)
    local length = Length(v)
    if length == 0 then
        return startPos
    end
    local easing = Easing(timer, a, b)
    return Add(startPos, Multiply(v, easing))
end