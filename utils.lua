require "Vector3"
require "Matrix3"

function Angle(v1, v2)
    local dot_product = Dot(v1, v2)
    local length_product = Length(v1) * Length(v2)
    if length_product == 0 then
        return 0
    end
    return math.acos(dot_product / length_product)
end

function SignedAngleAroundAxis(v1, v2, axis)
    -- Project v1 and v2 onto the plane perpendicular to axis
    local v1_proj = v1:projectToPlane(axis)
    local v2_proj = v2:projectToPlane(axis)

    -- Check for degenerate cases (vectors aligned with axis)
    local len_v1_proj = v1_proj:length()
    local len_v2_proj = v2_proj:length()
    if len_v1_proj < 1e-6 or len_v2_proj < 1e-6 then
        return 0 -- Angle undefined; no adjustment needed
    end

    -- Normalize the projections
    local u1 = v1_proj:normalize()
    local u2 = v2_proj:normalize()

    -- Compute signed angle using atan2
    local cross = u1:cross(u2)
    local theta = math.atan(cross:dot(axis), u1:dot(u2))
    return theta
end

function Clamp(value, min, max)
    return math.max(min, math.min(value, max))
end

function Round(num, decimal_places)
    local mult = 10^(decimal_places or 0)
    return math.floor(num * mult + 0.5) / mult
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

function applyConstraints(targetDirection, )