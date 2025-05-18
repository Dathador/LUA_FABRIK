require "Matrix3"

Vector3 = {}
Vector3.__index = Vector3

function Vector3:new(x, y, z)
    return setmetatable({x = x or 0, y = y or 0, z = z or 0}, Vector3)
end

function Vector3:copy()
    return Vector3:new(self.x, self.y, self.z)
end

function Vector3:length()
    return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
end

function Vector3:normalize()
    local len = self:length()
    if len > 0 then
        return Vector3:new(self.x / len, self.y / len, self.z / len)
    else
        return Vector3:new(0, 0, 0)
    end
end

function Vector3:dot(other)
    return self.x * other.x + self.y * other.y + self.z * other.z
end

function Vector3:cross(other)
    return Vector3:new(
        self.y * other.z - self.z * other.y,
        self.z * other.x - self.x * other.z,
        self.x * other.y - self.y * other.x
    )
end

function Vector3:add(other)
    return Vector3:new(self.x + other.x, self.y + other.y, self.z + other.z)
end

function Vector3:subtract(other)
    return Vector3:new(self.x - other.x, self.y - other.y, self.z - other.z)
end

function Vector3:scale(scalar)
    return Vector3:new(self.x * scalar, self.y * scalar, self.z * scalar)
end

function Vector3:distance(other)
    return self:subtract(other):length()
end

function Vector3:toString()
    return string.format("(%.3f, %.3f, %.3f)", self.x, self.y, self.z)
end

function Vector3:projectToPlane(normal)
    local d = self:dot(normal)
    return Vector3:new(
        self.x - d * normal.x,
        self.y - d * normal.y,
        self.z - d * normal.z
    )
end

function Vector3:RotateAround(angle, axis)
    local rotation = Matrix3:rotationFromAxisAngle(axis, angle)
    return rotation:multiply(self)
end

function Vector3:RotatePointAroundAxis(angle, axis, center)
    -- Rotate point p around axis through center by angle
    local v = self:subtract(center)
    local v_rot = v:RotateAround(angle, axis)
    return v_rot:add(center)
end