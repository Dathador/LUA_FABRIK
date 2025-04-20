require "utils"

bone = {}
bone.__index = bone

function bone:New(position, rotation, axisOfRotation, minAngle, maxAngle, length)
    local self = setmetatable({}, bone)
    self.position = position or {0, 0, 0}
    self.rotation = rotation or RotationMatrix(0, {0, 0, 1})
    self.axisOfRotation = axisOfRotation or {0, 0, 1}
    self.minAngle = minAngle or -math.pi
    self.maxAngle = maxAngle or  math.pi
    self.length = length or 1
    return self
end

