Matrix3 = {}
Matrix3.__index = Matrix3

function Matrix3:new(...)
    local args = {...}
    local m = setmetatable({}, Matrix3)
    
    if #args == 0 then
        -- Identity matrix
        m[1] = {1, 0, 0}
        m[2] = {0, 1, 0}
        m[3] = {0, 0, 1}
    elseif #args == 9 then
        -- Matrix from 9 values (row by row)
        m[1] = {args[1], args[2], args[3]}
        m[2] = {args[4], args[5], args[6]}
        m[3] = {args[7], args[8], args[9]}
    elseif #args == 3 then
        -- Matrix from 3 rows
        m[1] = {args[1][1], args[1][2], args[1][3]}
        m[2] = {args[2][1], args[2][2], args[2][3]}
        m[3] = {args[3][1], args[3][2], args[3][3]}
    end
    
    return m
end

function Matrix3:identity()
    return Matrix3:new(
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    )
end

function Matrix3:multiply(other)
    if type(other) == "table" and other.x then
        -- Matrix * Vector3
        return Vector3:new(
            self[1][1] * other.x + self[1][2] * other.y + self[1][3] * other.z,
            self[2][1] * other.x + self[2][2] * other.y + self[2][3] * other.z,
            self[3][1] * other.x + self[3][2] * other.y + self[3][3] * other.z
        )
    else
        -- Matrix * Matrix
        local result = Matrix3:new()
        for i = 1, 3 do
            for j = 1, 3 do
                result[i][j] = 0
                for k = 1, 3 do
                    result[i][j] = result[i][j] + self[i][k] * other[k][j]
                end
            end
        end
        return result
    end
end

function Matrix3:transpose()
    local result = Matrix3:new()
    for i = 1, 3 do
        for j = 1, 3 do
            result[i][j] = self[j][i]
        end
    end
    return result
end

function Matrix3:rotationFromAxisAngle(axis, angle)
    local x, y, z = axis.x, axis.y, axis.z
    local c, s = math.cos(angle), math.sin(angle)
    local t = 1 - c
    
    return Matrix3:new(
        t*x*x + c,    t*x*y - s*z,  t*x*z + s*y,
        t*x*y + s*z,  t*y*y + c,    t*y*z - s*x,
        t*x*z - s*y,  t*y*z + s*x,  t*z*z + c
    )
end

function Matrix3:getAxisAngle()
    -- Extract axis and angle from rotation matrix
    local angle = math.acos((self[1][1] + self[2][2] + self[3][3] - 1) / 2)
    
    if math.abs(angle) < 0.00001 or math.abs(angle - math.pi) < 0.00001 then
        -- For 0 or 180 degrees, the axis can be any perpendicular vector
        return Vector3:new(1, 0, 0), angle
    end
    
    local x = self[3][2] - self[2][3]
    local y = self[1][3] - self[3][1]
    local z = self[2][1] - self[1][2]
    local length = math.sqrt(x*x + y*y + z*z)
    
    return Vector3:new(x/length, y/length, z/length), angle
end

function Matrix3:toString()
    local str = ""
    for i = 1, 3 do
        str = str .. "["
        for j = 1, 3 do
            str = str .. string.format("%.3f", self[i][j])
            if j < 3 then str = str .. ", " end
        end
        str = str .. "]\n"
    end
    return str
end

-- Create rotation matrix from Euler angles (ZYX convention)
function Matrix3:fromEulerAngles(x, y, z)
    local cx, sx = math.cos(x), math.sin(x)
    local cy, sy = math.cos(y), math.sin(y)
    local cz, sz = math.cos(z), math.sin(z)
    
    return Matrix3:new(
        cy*cz, -cy*sz, sy,
        cx*sz + sx*sy*cz, cx*cz - sx*sy*sz, -sx*cy,
        sx*sz - cx*sy*cz, sx*cz + cx*sy*sz, cx*cy
    )
end

-- Extract Euler angles (in ZYX convention) from rotation matrix
function Matrix3:toEulerAngles()
    local y = math.asin(math.min(1, math.max(-1, self[1][3])))
    
    if math.abs(self[1][3]) < 0.99999 then
        local x = math.atan(-self[2][3], self[3][3])
        local z = math.atan(-self[1][2], self[1][1])
        return x, y, z
    else
        -- Gimbal lock case
        local x = 0
        local z = math.atan(self[2][1], self[2][2])
        return x, y, z
    end
end