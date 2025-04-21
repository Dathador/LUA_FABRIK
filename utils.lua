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

function MatrixVectorMult(m, v)
    local result = {}
    for i = 1, #m do
        result[i] = 0
        for j = 1, #v do
            result[i] = result[i] + m[i][j] * v[j]
        end
    end
    return result
end

function Distance(v1, v2)
    local squared_distance = 0
    for i = 1, #v1 do
        squared_distance = squared_distance + (v1[i] - v2[i])^2
    end
    return math.sqrt(squared_distance)
end

function Subtract(v1, v2)
    local result = {}
    for i = 1, #v1 do
        result[i] = v1[i] - v2[i]
    end
    return result
end

function Add(v1, v2)
    local result = {}
    for i = 1, #v1 do
        result[i] = v1[i] + v2[i]
    end
    return result
end

function Length(v)
    local squared_length = 0
    for i = 1, #v do
        squared_length = squared_length + v[i]^2
    end
    return math.sqrt(squared_length)
end

function UnitVector(v)
    local length = Length(v)
    if length == 0 then
        return {0, 0, 0}
    end
    local result = {}
    for i = 1, #v do
        result[i] = v[i] / length
    end
    return result
end

function Multiply(v, scalar)
    local result = {}
    for i = 1, #v do
        result[i] = v[i] * scalar
    end
    return result
end

function Dot(v1, v2)
    local result = 0
    for i = 1, #v1 do
        result = result + v1[i] * v2[i]
    end
    return result
end

function Cross(v1, v2)
    if #v1 == 2 and #v2 == 2 then
        return v1[1] * v2[2] - v1[2] * v2[1]
    elseif #v1 == 3 and #v2 == 3 then
        return {
            v1[2] * v2[3] - v1[3] * v2[2],
            v1[3] * v2[1] - v1[1] * v2[3],
            v1[1] * v2[2] - v1[2] * v2[1]
        }
    else
        error("Cross product is only defined for 2D or 3D vectors")
    end
end

function Angle(v1, v2)
    local dot_product = Dot(v1, v2)
    local length_product = Length(v1) * Length(v2)
    if length_product == 0 then
        return 0
    end
    return math.acos(dot_product / length_product)
end

function ProjectToPlane(v, normal)
    return Subtract(v, Multiply(normal, Dot(v, normal)))
end

function SignedAngleAroundAxis(v1, v2, axis)
    -- Project v1 and v2 onto the plane perpendicular to axis
    local v1_proj = ProjectToPlane(v1, axis)
    local v2_proj = ProjectToPlane(v2, axis)

    -- Check for degenerate cases (vectors aligned with axis)
    local len_v1_proj = Length(v1_proj)
    local len_v2_proj = Length(v2_proj)
    if len_v1_proj < 1e-6 or len_v2_proj < 1e-6 then
        return 0 -- Angle undefined; no adjustment needed
    end

    -- Normalize the projections
    local u1 = Multiply(v1_proj, 1 / len_v1_proj)
    local u2 = Multiply(v2_proj, 1 / len_v2_proj)

    -- Compute signed angle using atan2
    local cross = Cross(u1, u2)
    local theta = math.atan(Dot(cross, axis), Dot(u1, u2))
    return theta
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

function RotatePointAroundAxis(v, angle, axis, center)
    -- Rotate point p around axis through center by angle
    local v = Subtract(v, center)
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